#
# This file is part of raptor-camd.
#
# raptor-camd is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# raptor-camd is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with raptor-camd.  If not, see <http://www.gnu.org/licenses/>.

"""Helper process for preparing and saving fits images"""

# pylint: disable=too-many-arguments
# pylint: disable=too-many-branches

import os.path
import shutil
from astropy.io import fits
import astropy.units as u
import numpy as np
from warwick.observatory.common import daemons, log
from .constants import CoolerMode


def window_sensor_region(region, window):
    """Calculate new region coordinates when cropped to a given window"""
    x1 = max(0, region[0] - window[0])
    x2 = min(region[1] - window[0], window[1] - window[0])
    y1 = max(0, region[2] - window[2])
    y2 = min(region[3] - window[2], window[3] - window[2])
    if x1 > x2 or y1 > y2:
        return None

    return [x1, x2, y1, y2]


def format_sensor_region(region):
    """Format a 0-indexed region as a 1-indexed fits region"""
    return f'[{region[0] + 1}:{region[1] + 1},{region[2] + 1}:{region[3] + 1}]'


def output_process(process_queue, processing_framebuffer, processing_framebuffer_offsets, stop_signal,
                   camera_id, camera_device_id, header_card_capacity, output_path, log_name,
                   pipeline_daemon_name, pipeline_handover_timeout, software_version):
    """
    Helper process to save frames to disk.
    This uses a process (rather than a thread) to avoid the GIL bottlenecking throughput,
    and multiple worker processes allow frames to be handled in parallel.
    """
    pipeline_daemon = getattr(daemons, pipeline_daemon_name)
    while True:
        frame = process_queue.get()

        data = np.frombuffer(processing_framebuffer, dtype=np.uint16,
                             offset=frame['data_offset'], count=frame['data_height'] * frame['data_width']) \
            .reshape((frame['data_height'], frame['data_width'])).copy()
        processing_framebuffer_offsets.put(frame['data_offset'])

        date_end = frame['timestamp'] - frame['readout_time'] * u.s
        date_start = date_end - frame['exposure'] * u.s

        if frame['cooler_setpoint'] is not None:
            setpoint_header = ('TEMP-SET', frame['cooler_setpoint'], '[deg c] cmos temperature set point')
        else:
            setpoint_header = ('COMMENT', ' TEMP-SET not available', '')

        header = [
            (None, None, None),
            ('COMMENT', ' ---                DATE/TIME                --- ', ''),
            ('DATE-OBS', date_start.strftime('%Y-%m-%dT%H:%M:%S.%f'), '[utc] estimated exposure start time'),
            ('DATE-END', date_end.strftime('%Y-%m-%dT%H:%M:%S.%f'), '[utc] estimated exposure end time'),
            ('TIME-SRC', 'NTP', 'DATE-OBS is estimated from NTP-synced PC clock'),
            ('EXPTIME', round(frame['exposure'], 3), '[s] actual exposure length'),
            ('EXPCADNC', round(frame['frameperiod'], 3), '[s] exposure cadence'),
            ('PC-RDEND', frame['read_end_time'].strftime('%Y-%m-%dT%H:%M:%S.%f'),
             '[utc] local PC time when readout completed'),
            (None, None, None),
            ('COMMENT', ' ---           CAMERA INFORMATION            --- ', ''),
            ('CAMID', camera_id, 'camera identifier'),
            ('CAMERA', camera_device_id, 'camera model and serial number'),
            ('CAMSWVER', software_version, 'camera server software version'),
            ('CAMDRV', frame['camera_driver'], ''),
            ('CAMSDK', frame['camera_library'], 'EPIX SDK version'),
            ('CAMFPGA', frame['fpga_version'], 'camera FPGA firmware version'),
            ('CAMMICRO', frame['micro_version'], 'camera microcontroller firmware version'),
            ('CAMGRAB', frame['grabber_model'], 'camera frame grabber model'),
            ('CAMBDATE', frame['camera_build_date'], 'camera build date'),
            ('CAMBCODE', frame['camera_build_code'], 'camera build code'),
            ('CAM-TEMP', round(frame['sensor_temperature'], 2),
             '[deg c] sensor temperature at end of exposure'),
            ('CAM-PCBT', round(frame['pcb_temperature'], 2),
             '[deg c] pcb temperature at end of exposure'),
            ('TEMP-MOD', CoolerMode.label(frame['cooler_mode']), 'temperature control mode'),
            setpoint_header,
            ('TEMP-LCK', frame['cooler_mode'] == CoolerMode.Locked, 'cmos temperature is locked to set point'),
            ('CAM-XBIN', 1, '[px] x binning'),
            ('CAM-YBIN', 1, '[px] y binning'),
            ('CAM-WIND', f'[1:{frame["data_width"]},1:{frame["data_height"]}',
             '[x1:x2,y1:y2] readout region (detector coords)'),
            ('IMAG-RGN', f'[1:{frame["data_width"]},1:{frame["data_height"]}',
             '[x1:x2,y1:y2] image region (image coords)'),
            ('FIELD', frame['field'], 'frame count since camera initialization'),
            ('EXPCNT', frame['exposure_count'], 'running exposure count since EXPCREF'),
            ('EXPCREF', frame['exposure_count_reference'], 'date the exposure counter was reset'),
        ]

        hdu = fits.PrimaryHDU(data)

        # Using Card and append() to force comment cards to be placed inline
        for h in header:
            hdu.header.append(fits.Card(h[0], h[1], h[2]), end=True)

        # Pad with sufficient blank cards that pipelined won't need to allocate extra header blocks
        padding = max(0, header_card_capacity - len(hdu.header) - 1)
        for _ in range(padding):
            hdu.header.append(fits.Card(None, None, None), end=True)

        # Save errors shouldn't interfere with preview updates, so we use a separate try/catch
        try:
            filename = f'{camera_id}-{frame["exposure_count"]:08d}.fits'
            path = os.path.join(output_path, filename)

            # Simulate an atomic write by writing to a temporary file then renaming
            hdu.writeto(path + '.tmp', overwrite=True)
            shutil.move(path + '.tmp', path)
            print('Saving temporary frame: ' + filename)

        except Exception as e:
            stop_signal.value = True
            log.error(log_name, 'Failed to save temporary frame (' + str(e) + ')')
            continue

        # Hand frame over to the pipeline
        # This may block if the pipeline is busy
        try:
            with pipeline_daemon.connect(pipeline_handover_timeout) as pipeline:
                pipeline.notify_frame(camera_id, filename)
        except Exception as e:
            stop_signal.value = True
            log.error(log_name, 'Failed to hand frame to pipeline (' + str(e) + ')')
