#
# This file is part of the Robotic Observatory Control Kit (rockit)
#
# rockit is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# rockit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with rockit.  If not, see <http://www.gnu.org/licenses/>.

"""Helper process for interfacing with the EPIX SDK"""

# pylint: disable=too-many-arguments
# pylint: disable=too-many-instance-attributes
# pylint: disable=too-many-return-statements
# pylint: disable=too-many-branches
# pylint: disable=too-many-statements
# pylint: disable=broad-exception-raised
# pylint: disable=too-many-nested-blocks

from ctypes import byref, c_char, c_char_p, c_double, c_int, c_uint8, c_uint16, c_uint64, c_void_p
from ctypes import create_string_buffer, POINTER, Structure
import json
import pathlib
import platform
import sys
import threading
import time
import traceback
from astropy.time import Time
import astropy.units as u
import numpy as np
import Pyro4
from rockit.common import log
from .constants import CommandStatus, CameraStatus, CoolerMode


def format_hex(data):
    return ' '.join([f'0x{d:02X}' for d in data])


class ManufacturerData(Structure):
    _pack_ = 1
    _fields_ = [
        ("serial", c_uint16.__ctype_le__),
        ("build_day", c_uint8),
        ("build_month", c_uint8),
        ("build_year", c_uint8),
        ("build_code", 5 * c_char),
        ("adc_cal_0", c_uint16.__ctype_le__),
        ("adc_cal_40", c_uint16.__ctype_le__),
        ("dac_cal_0", c_uint16.__ctype_le__),
        ("dac_cal_40", c_uint16.__ctype_le__)
    ]


class RaptorInterface:
    def __init__(self, config, processing_queue,
                 processing_framebuffer, processing_framebuffer_offsets,
                 processing_stop_signal):
        self._config = config

        self._handle = c_void_p()
        self._xclib = None
        self._lock = threading.Lock()
        self._tick_origin = None

        self._camera_library = ''
        self._grabber_model = ''
        self._micro_version = ''
        self._fpga_version = ''
        self._camera_serial = ''
        self._adc_slope = 0
        self._adc_offset = 0
        self._dac_slope = 0
        self._dac_offset = 0
        self._readout_time = 0
        self._readout_width = 0
        self._readout_height = 0

        self._chiller_error = False
        self._cooler_mode = CoolerMode.Unknown
        self._cooler_setpoint = config.cooler_setpoint
        self._sensor_temperature = 0
        self._pcb_temperature = 0

        self._exposure_time = 0

        # Limit and number of frames acquired during the next sequence
        # Set to 0 to run continuously
        self._sequence_frame_limit = 0

        # Number of frames acquired this sequence
        self._sequence_frame_count = 0

        # Time that the latest frame in the exposure was started
        self._sequence_exposure_start_time = Time.now()

        # Information for building the output filename
        self._output_directory = pathlib.Path(config.output_path)
        self._output_frame_prefix = config.output_prefix

        # Persistent frame counters
        self._counter_filename = config.expcount_path
        try:
            with open(self._counter_filename, 'r', encoding='ascii') as infile:
                data = json.load(infile)
                self._exposure_count = data['exposure_count']
                self._exposure_count_reference = data['exposure_reference']
        except Exception:
            now = Time.now().strftime('%Y-%m-%d')
            self._exposure_count = 0
            self._exposure_count_reference = now

        # Thread that runs the exposure sequence
        # Initialized by start() method
        self._acquisition_thread = None

        # Signal that the exposure sequence should be terminated
        # at end of the current frame
        self._stop_acquisition = False

        # Subprocess for processing acquired frames
        self._processing_queue = processing_queue
        self._processing_stop_signal = processing_stop_signal

        # A large block of shared memory for sending frame data to the processing workers
        self._processing_framebuffer = processing_framebuffer

        # A queue of memory offsets that are available to write frame data into
        # Offsets are popped from the queue as new frames are written into the
        # frame buffer, and pushed back on as processing is complete
        self._processing_framebuffer_offsets = processing_framebuffer_offsets

    @property
    def is_acquiring(self):
        return self._acquisition_thread is not None and self._acquisition_thread.is_alive()

    def update_cooler(self):
        """Polls and updates cooler status"""
        try:
            with self._lock:
                # Query temperature status
                high = self._serial_command(b'\x53\x01\x03\x01\x05\x6E', 1)[0]
                low = self._serial_command(b'\x53\x01\x03\x01\x05\x6F', 1)[0]
                self._sensor_temperature = (256 * (high & 0x0F) + low) * self._adc_slope + self._adc_offset

                high = self._serial_command(b'\x53\x01\x03\x01\x05\x70', 1)[0]
                low = self._serial_command(b'\x53\x01\x03\x01\x05\x71', 1)[0]
                self._pcb_temperature = int.from_bytes([(high << 4) + (low >> 4)], byteorder='big', signed=True)
                self._pcb_temperature += int.from_bytes([low & 0xF], byteorder='big', signed=True) * 0.0625

                if self._cooler_setpoint is not None:
                    delta = abs(self._sensor_temperature - self._cooler_setpoint)
                    self._cooler_mode = CoolerMode.Locked if delta < 0.5 else CoolerMode.Locking
                else:
                    self._cooler_mode = CoolerMode.Off
        except:
            self._cooler_mode = CoolerMode.Unknown

        if self._config.chiller_daemon and self._cooler_mode != CoolerMode.Off:
            try:
                with self._config.chiller_daemon.connect() as chiller:
                    chiller.notify_camera_cooling_active()
                self._chiller_error = False
            except Exception:
                if not self._chiller_error:
                    log.error(self._config.log_name, 'Failed to notify chiller daemon of cooling status')
                    self._chiller_error = True

    def _serial_command(self, data, response_length=0, timeout=5):
        chk = 0x50
        for b in data:
            chk ^= b

        send = data + bytes([0x50, chk])

        ret = self._xclib.pxd_serialWrite(1, 0, send, len(send))
        if ret < 0:
            raise Exception(f'failed to send command: {format_hex(send)}')

        response = create_string_buffer(response_length + 2)
        wait_start = Time.now()
        while True:
            available = self._xclib.pxd_serialRead(1, 0, None, 0)
            if available >= response_length + 2:
                ret = self._xclib.pxd_serialRead(1, 0, response, response_length + 2)
                if ret != response_length + 2:
                    raise Exception('failed to read command response')

                # Validate the ACK and checksum
                if response.raw[-2] != 0x50:
                    raise Exception(f'unexpected ACK value 0x{response.raw[-2]:02x} != 0x50')

                if response.raw[-1] != chk:
                    raise Exception(f'unexpected ACK value 0x{response.raw[-1]:02x} != 0x{chk:02x}')

                return response.raw[:-2]

            if Time.now() - wait_start > timeout * u.s:
                raise Exception('timeout while waiting for command response')

            time.sleep(0.001)

    def __run_exposure_sequence(self, quiet):
        """Worker thread that acquires frames and their times.
           Tagged frames are pushed to the acquisition queue
           for further processing on another thread"""
        framebuffer_slots = 0
        try:
            with self._lock:
                # Low gain mode has a maximum exposure time of 1.048.57ms
                # Split longer requested exposures into multiple sub-exposures
                coadd_count = 1
                while self._exposure_time / coadd_count > 1.048:
                    coadd_count += 1

                exposure_time = self._exposure_time / coadd_count

                # Set frame period
                period = int((exposure_time + self._readout_time) * 70e6).to_bytes(4, 'big')
                self._serial_command(b'\x53\x00\x03\x01\xDD' + period[0:1])
                self._serial_command(b'\x53\x00\x03\x01\xDE' + period[1:2])
                self._serial_command(b'\x53\x00\x03\x01\xDF' + period[2:3])
                self._serial_command(b'\x53\x00\x03\x01\xE0' + period[3:4])

                # Set exposure time
                exp = int(exposure_time * 1e6).to_bytes(4, 'big')
                self._serial_command(b'\x53\x00\x03\x01\xEE' + exp[0:1])
                self._serial_command(b'\x53\x00\x03\x01\xEF' + exp[1:2])
                self._serial_command(b'\x53\x00\x03\x01\xF0' + exp[2:3])
                self._serial_command(b'\x53\x00\x03\x01\xF1' + exp[3:4])

            # Prepare the framebuffer offsets
            if not self._processing_framebuffer_offsets.empty():
                log.error(self._config.log_name, 'Frame buffer offsets queue is not empty!')
                return

            dma_buffer_count = self._xclib.pxd_imageZdim(1)

            # Allocate buffer space for 32-bit data to support coadding more than 16 frames
            # 16 bit images only use the first half of each buffer slot
            pixel_count = self._readout_width * self._readout_height
            frame_size = 4 * pixel_count
            output_buffer_count = len(self._processing_framebuffer) // frame_size
            if output_buffer_count != dma_buffer_count:
                size = dma_buffer_count * frame_size
                print(f'warning: framebuffer_bytes should be set to {size} for optimal performance')

            offset = 0
            framebuffer_slot_arrays = {}
            framebuffer_dtype = np.uint32 if coadd_count > 15 else np.uint16
            while offset + frame_size <= len(self._processing_framebuffer):
                self._processing_framebuffer_offsets.put(offset)
                framebuffer_slot_arrays[offset] = np.frombuffer(self._processing_framebuffer,
                                                                offset=offset, dtype=framebuffer_dtype,
                                                                count=pixel_count)
                offset += frame_size
                framebuffer_slots += 1
                if framebuffer_slots == dma_buffer_count:
                    break

            with self._lock:
                last_buffer = self._xclib.pxd_capturedBuffer(1)
                # Queue all available buffers to start sequence
                for i in range(8):
                    self._xclib.pxd_quLive(1, i + 1)

            readout_buffer = bytearray(2 * pixel_count)
            readout_array = np.frombuffer(readout_buffer, dtype=np.uint16)
            readout_cdata = (c_uint16 * pixel_count).from_buffer(readout_buffer)

            aborted = False
            while not self._stop_acquisition and not self._processing_stop_signal.value:
                self._sequence_exposure_start_time = Time.now()
                framebuffer_offset = self._processing_framebuffer_offsets.get()

                coadd_index = 0
                coadd_first_field = 0
                read_end_time = Time.now()
                while True:
                    # Wait for frame to become available
                    while True:
                        buffer = self._xclib.pxd_capturedBuffer(1)
                        if buffer != last_buffer or self._stop_acquisition or self._processing_stop_signal.value:
                            break
                        time.sleep(0.001)

                    if self._stop_acquisition or self._processing_stop_signal.value:
                        aborted = True
                        break

                    last_buffer = buffer

                    field = self._xclib.pxd_buffersFieldCount(1, buffer)
                    if coadd_index > 0 and field != coadd_first_field + coadd_index:
                        log.warning(self._config.log_name,
                                    f'sub-exposure {coadd_index + 1} / {coadd_count} dropped; restarting frame')
                        coadd_index = 0

                    ret = self._xclib.pxd_readushort(1, buffer, 0, 0, self._readout_width, self._readout_height,
                                                     readout_cdata, pixel_count, b'GREY')
                    self._xclib.pxd_quLive(1, buffer)
                    if ret < 0:
                        print(f'Failed to read frame data: {self._xclib.pxd_mesgErrorCode(ret)}')
                        log.warning(self._config.log_name,
                                    f'sub-exposure {coadd_index + 1} / {coadd_count} failed; restarting frame')
                        coadd_index = 0
                        continue

                    if platform.system() == 'Windows':
                        ticks = c_uint64()
                        ret = self._xclib.pxd_buffersSysTicks2(1, buffer, byref(ticks))
                        if ret < 0:
                            print(f'Failed to read frame timestamp: {self._xclib.pxd_mesgErrorCode(ret)}')
                            read_end_time = Time.now()
                        else:
                            read_end_time = self._tick_origin + 100 * ticks.value * u.nanosecond
                    else:
                        read_end_time = Time.now()

                    if coadd_index == 0:
                        framebuffer_slot_arrays[framebuffer_offset][:] = readout_array[:]
                        coadd_first_field = field
                    else:
                        framebuffer_slot_arrays[framebuffer_offset] += readout_array

                    coadd_index += 1
                    if coadd_index == coadd_count:
                        break

                if aborted:
                    # Return unused slot back to the queue to simplify cleanup
                    self._processing_framebuffer_offsets.put(framebuffer_offset)
                    break

                self._processing_queue.put({
                    'data_offset': framebuffer_offset,
                    'data_width': self._readout_width,
                    'data_height': self._readout_height,
                    'exposure': float(self._exposure_time),
                    'subexposure': float(exposure_time),
                    'coadd_count': coadd_count,
                    'frameperiod': coadd_count * (exposure_time + self._readout_time),
                    'field': coadd_first_field,
                    'readout_time': self._readout_time,
                    'read_end_time': read_end_time,
                    'cooler_mode': self._cooler_mode,
                    'cooler_setpoint': self._cooler_setpoint,
                    'sensor_temperature': self._sensor_temperature,
                    'pcb_temperature': self._pcb_temperature,
                    'camera_library': self._camera_library,
                    'grabber_model': self._grabber_model,
                    'micro_version': self._micro_version,
                    'fpga_version': self._fpga_version,
                    'exposure_count': self._exposure_count,
                    'exposure_count_reference': self._exposure_count_reference
                })

                self._exposure_count += 1
                self._sequence_frame_count += 1

                # Continue exposure sequence?
                if 0 < self._sequence_frame_limit <= self._sequence_frame_count:
                    self._stop_acquisition = True
        except Exception as e:
            print(e)
        finally:
            with self._lock:
                self._xclib.pxd_goAbortLive(1)

            # Save updated counts to disk
            with open(self._counter_filename, 'w', encoding='ascii') as outfile:
                json.dump({
                    'exposure_count': self._exposure_count,
                    'exposure_reference': self._exposure_count_reference,
                }, outfile)

            # Wait for processing to complete
            for _ in range(framebuffer_slots):
                self._processing_framebuffer_offsets.get()

            if not quiet:
                log.info(self._config.log_name, 'Exposure sequence complete')
            self._stop_acquisition = False

    def initialize(self):
        """Connects to the camera driver"""
        print('initializing frame grabber')
        initialized = False
        with self._lock:
            pixci_args = b'-CQ 8'
            # pylint: disable=import-outside-toplevel
            if platform.system() == 'Windows':
                from ctypes import WinDLL
                self._xclib = WinDLL(r'C:\Program Files\EPIX\XCLIB\lib\xclibw64.dll')

                # Timestamp frames using the "KeQueryInterruptTime" kernel counter
                # "KeQuerySystemTimePrecise" (TI = 5) does not appear to work!
                pixci_args += b' -TI 3'

                # The interrupt time counter increments in 100 nanosecond intervals
                # since system boot
                ticks = c_uint64()
                origin = Time.now()
                WinDLL('kernelbase').QueryInterruptTime(byref(ticks))
                self._tick_origin = origin - 100 * ticks.value * u.nanosecond
            else:
                from ctypes import CDLL
                self._xclib = CDLL('/usr/local/xclib/lib/xclib_x86_64.so')
            # pylint: enable=import-outside-toplevel

            self._xclib.pxd_PIXCIopen.argtypes = [c_char_p, c_char_p, c_char_p]
            self._xclib.pxd_serialWrite.argtypes = [c_int, c_int, POINTER(c_char), c_int]
            self._xclib.pxd_serialConfigure.argtypes = [
                c_int, c_int, c_double, c_int, c_int, c_int, c_int, c_int, c_int]
            self._xclib.pxd_readushort.argtypes = [
                c_int, c_int, c_int, c_int, c_int, c_int, c_void_p, c_int, c_char_p]
            self._xclib.pxd_mesgErrorCode.restype = c_char_p
            self._xclib.pxd_infoDriverId.restype = c_char_p
            self._xclib.pxd_infoLibraryId.restype = c_char_p

            try:
                ret = self._xclib.pxd_PIXCIopen(pixci_args,
                                                None,
                                                self._config.camera_config_path.encode('ascii'))
                if ret != 0:
                    print(f'Failed to open PIXCI: {self._xclib.pxd_mesgErrorCode(ret)}')
                    return 1

                self._camera_library = self._xclib.pxd_infoLibraryId(1).decode('ascii')
                grabber_model = self._xclib.pxd_infoModel(1)
                if grabber_model == 0x0030:
                    self._grabber_model = 'PIXCI_E8'
                else:
                    self._grabber_model = 'UNKNOWN ({grabber_model:04x})'

                ret = self._xclib.pxd_serialConfigure(1, 0, 115200, 8, 0, 1, 0, 0, 0)
                if ret != 0:
                    print(f'Failed to configure PIXCI serial: {self._xclib.pxd_mesgErrorCode(ret)}')
                    return 1

                self._xclib.pxd_serialFlush(1, 0, 1, 1)

                # Wait for FPGA to boot
                # ACK and checksum may or may not be enabled depending
                # on whether the camera has been power cycled
                while True:
                    ret = self._xclib.pxd_serialWrite(1, 0, b'\x49\x50\x19', 3)
                    if ret < 0:
                        raise Exception(f'failed to send status query {ret}')

                    ignore_bytes = 0
                    fpga_booted = False
                    wait_start = Time.now()
                    while True:
                        available = self._xclib.pxd_serialRead(1, 0, None, 0)
                        if available >= 1:
                            buf = create_string_buffer(1)
                            ret = self._xclib.pxd_serialRead(1, 0, buf, 1)
                            if ret != 1:
                                raise Exception('failed to read status')

                            status = buf.raw[0]

                            # camera will send an ACK byte
                            if (status & 0x10) != 0:
                                ignore_bytes += 1

                            # camera will send a checksum byte
                            if (status & 0x40) != 0:
                                ignore_bytes += 1

                            if (status & 0x04) != 0:
                                fpga_booted = True
                            break

                        if Time.now() - wait_start > 1 * u.s:
                            raise Exception('timeout while waiting for status')

                        time.sleep(0.001)

                    if ignore_bytes > 0:
                        wait_start = Time.now()
                        while True:
                            available = self._xclib.pxd_serialRead(1, 0, None, 0)
                            if available >= ignore_bytes:
                                buf = create_string_buffer(ignore_bytes)
                                ret = self._xclib.pxd_serialRead(1, 0, buf, ignore_bytes)
                                if ret != ignore_bytes:
                                    raise Exception('failed to read ignored bytes')
                                break

                            if Time.now() - wait_start > 1 * u.s:
                                raise Exception('timeout while waiting for ignored bytes')

                            time.sleep(0.001)

                    if fpga_booted:
                        break

                # Enable command ACK, checksum, NVM access
                self._serial_command(b'\x4F\x53')

                ret = self._serial_command(b'\x56', 2)
                self._micro_version = f'{ret[0]}.{ret[1]}'

                # Wait for FPGA to load NUC tables
                time.sleep(5.)

                major = self._serial_command(b'\x53\x01\x03\x01\x05\x7E', 1)[0]
                minor = self._serial_command(b'\x53\x01\x03\x01\x05\x7F', 1)[0]
                self._fpga_version = f'{major:d}.{minor:d}'

                data = self._serial_command(b'\x53\x05\x05\x12\x03\x00\x00\x00', 18)
                data = ManufacturerData.from_buffer_copy(data)

                self._camera_serial = f'{data.serial}'
                if data.serial != self._config.camera_serial:
                    print(f'Unknown camera serial: {data.serial} != {self._config.camera_serial}')
                    return CommandStatus.Failed

                self._adc_slope = 40.0 / (data.adc_cal_40 - data.adc_cal_0)
                self._adc_offset = -self._adc_slope * data.adc_cal_0
                self._dac_slope = (data.dac_cal_40 - data.dac_cal_0) / 40.0
                self._dac_offset = data.dac_cal_0

                # Disable automatic gain calculation mode
                self._serial_command(b'\x53\x00\x03\x01\x00\x00')

                # Set digital gain to minimum value (256)
                self._serial_command(b'\x53\x00\x03\x01\xC6\x01')
                self._serial_command(b'\x53\x00\x03\x01\xC7\x00')

                # Disable non-uniformity corrections
                self._serial_command(b'\x53\x00\x03\x01\xF9\x4C')

                # Low gain with internal triggering
                self._serial_command(b'\x53\x00\x03\x01\xF2\x00')

                # Set exposure time to 1 second while idling
                self._serial_command(b'\x53\x00\x03\x01\xDD\x04')
                self._serial_command(b'\x53\x00\x03\x01\xDE\x3C')
                self._serial_command(b'\x53\x00\x03\x01\xDF\x23')
                self._serial_command(b'\x53\x00\x03\x01\xE0\x10')

                self._serial_command(b'\x53\x00\x03\x01\xEE\x00')
                self._serial_command(b'\x53\x00\x03\x01\xEF\x0F')
                self._serial_command(b'\x53\x00\x03\x01\xF0\x42')
                self._serial_command(b'\x53\x00\x03\x01\xF1\x40')
                self._exposure_time = 1
                # Actual readout time is 7.52ms, but we round to
                # 8ms to avoid frame data stalls and to match the
                # timestamp resolution
                self._readout_time = 0.008

                # Enable cooling
                setpoint = int(self._config.cooler_setpoint * self._dac_slope + self._dac_offset).to_bytes(2, 'big')
                self._serial_command(b'\x53\x00\x03\x01\xFB' + setpoint[0:1])
                self._serial_command(b'\x53\x00\x03\x01\xFA' + setpoint[1:2])

                # Enable fan cooling if not connected to a chiller
                self._serial_command(b'\x53\x00\x03\x01\x00' + (b'\x01' if self._config.chiller_daemon else b'\x05'))

                self._cooler_setpoint = self._config.cooler_setpoint

                self._readout_width = self._xclib.pxd_imageXdim()
                self._readout_height = self._xclib.pxd_imageYdim()

                initialized = True

                return CommandStatus.Succeeded
            except Exception:
                traceback.print_exc(file=sys.stdout)
                return CommandStatus.Failed
            finally:
                # Clean up on failure
                if not initialized:
                    if self._xclib is not None:
                        self._xclib.pxd_PIXCIclose()
                        self._xclib = None

                    log.error(self._config.log_name, 'Failed to initialize camera')
                else:
                    log.info(self._config.log_name, 'Initialized camera')

    def set_target_temperature(self, temperature, quiet):
        """Set the target camera temperature"""
        if temperature is not None and (temperature < -50 or temperature > 30):
            return CommandStatus.TemperatureOutsideLimits

        try:
            with self._lock:
                if temperature is not None:
                    setpoint = int(temperature * self._dac_slope + self._dac_offset).to_bytes(2, 'big')
                    self._serial_command(b'\x53\x00\x03\x01\xFB' + setpoint[0:1])
                    self._serial_command(b'\x53\x00\x03\x01\xFA' + setpoint[1:2])

                    # Enable TEC and fan if needed
                    if self._cooler_setpoint is None:
                        # Enable fan cooling if not connected to a chiller
                        self._serial_command(b'\x53\x00\x03\x01\x00' +
                                             b'\x01' if self._config.chiller_daemon else b'\x05')
                else:
                    # Disable TEC and fan if needed
                    if self._cooler_setpoint is not None:
                        self._serial_command(b'\x53\x00\x03\x01\x00\x00')
        except:
            return CommandStatus.Failed

        self._cooler_setpoint = temperature
        if not quiet:
            log.info(self._config.log_name, f'Target temperature set to {temperature}')

        return CommandStatus.Succeeded

    def set_exposure(self, exposure, quiet):
        """Set the camera exposure time"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        self._exposure_time = exposure
        if not quiet:
            log.info(self._config.log_name, f'Exposure time set to {exposure:.3f}s')

        return CommandStatus.Succeeded

    @Pyro4.expose
    def start_sequence(self, count, quiet):
        """Starts an exposure sequence with a set number of frames, or 0 to run until stopped"""
        if self.is_acquiring:
            return CommandStatus.CameraNotIdle

        self._sequence_frame_limit = count
        self._sequence_frame_count = 0
        self._stop_acquisition = False
        self._processing_stop_signal.value = False

        self._acquisition_thread = threading.Thread(
            target=self.__run_exposure_sequence,
            args=(quiet,), daemon=True)
        self._acquisition_thread.start()

        if not quiet:
            count_msg = 'until stopped'
            if count == 1:
                count_msg = '1 frame'
            elif count > 1:
                count_msg = f'{count} frames'

            log.info(self._config.log_name, f'Starting exposure sequence ({count_msg})')

        return CommandStatus.Succeeded

    @Pyro4.expose
    def stop_sequence(self, quiet):
        """Stops any active exposure sequence"""
        if not self.is_acquiring or self._stop_acquisition:
            return CommandStatus.CameraNotAcquiring

        if not quiet:
            log.info(self._config.log_name, 'Aborting exposure sequence')

        self._sequence_frame_count = 0
        self._stop_acquisition = True

        return CommandStatus.Succeeded

    def report_status(self):
        """Returns a dictionary containing the current camera state"""
        # Estimate the current frame progress based on the time delta
        exposure_progress = 0
        sequence_frame_count = self._sequence_frame_count
        state = CameraStatus.Idle

        if self.is_acquiring:
            state = CameraStatus.Acquiring
            if self._stop_acquisition:
                state = CameraStatus.Aborting
            else:
                if self._sequence_exposure_start_time is not None:
                    exposure_progress = (Time.now() - self._sequence_exposure_start_time).to(u.s).value
                    if exposure_progress >= self._exposure_time:
                        state = CameraStatus.Reading

        return {
            'state': state,
            'cooler_mode': self._cooler_mode,
            'cooler_setpoint': self._cooler_setpoint,
            'sensor_temperature': self._sensor_temperature,
            'pcb_temperature': self._pcb_temperature,
            'temperature_locked': self._cooler_mode == CoolerMode.Locked,  # used by opsd
            'exposure_time': self._exposure_time,
            'exposure_progress': exposure_progress,
            'sequence_frame_limit': self._sequence_frame_limit,
            'sequence_frame_count': sequence_frame_count,
        }

    def shutdown(self):
        """Disconnects from the camera driver"""
        # Complete the current exposure
        if self._acquisition_thread is not None:
            print('shutdown: waiting for acquisition to complete')
            self._stop_acquisition = True
            self._acquisition_thread.join()

        with self._lock:
            print('shutdown: disconnecting driver')
            self._xclib.pxd_PIXCIclose()
            self._xclib = None

        log.info(self._config.log_name, 'Shutdown camera')
        return CommandStatus.Succeeded


def raptor_process(camd_pipe, config, processing_queue, processing_framebuffer, processing_framebuffer_offsets,
                   stop_signal):
    cam = RaptorInterface(config, processing_queue, processing_framebuffer, processing_framebuffer_offsets, stop_signal)
    ret = cam.initialize()

    if ret == CommandStatus.Succeeded:
        cam.update_cooler()

    camd_pipe.send(ret)
    if ret != CommandStatus.Succeeded:
        return

    try:
        last_cooler_update = Time.now()
        while True:
            temperature_dirty = False
            if camd_pipe.poll(timeout=1):
                c = camd_pipe.recv()
                command = c['command']
                args = c['args']

                if command == 'temperature':
                    temperature_dirty = True
                    camd_pipe.send(cam.set_target_temperature(args['temperature'], args['quiet']))
                elif command == 'exposure':
                    camd_pipe.send(cam.set_exposure(args['exposure'], args['quiet']))
                elif command == 'start':
                    camd_pipe.send(cam.start_sequence(args['count'], args['quiet']))
                elif command == 'stop':
                    camd_pipe.send(cam.stop_sequence(args['quiet']))
                elif command == 'status':
                    camd_pipe.send(cam.report_status())
                elif command == 'shutdown':
                    break
                else:
                    print(f'unhandled command: {command}')
                    camd_pipe.send(CommandStatus.Failed)

            dt = Time.now() - last_cooler_update
            if temperature_dirty or dt > config.cooler_update_delay * u.s:
                cam.update_cooler()
                last_cooler_update = Time.now()
    except Exception:
        traceback.print_exc(file=sys.stdout)
        camd_pipe.send(CommandStatus.Failed)

    camd_pipe.close()
    cam.shutdown()
