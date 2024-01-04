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

"""client command input handlers"""

import Pyro4
from rockit.common import TFmt
from .config import Config
from .constants import CommandStatus, CameraStatus, CoolerMode


def run_client_command(config_path, usage_prefix, args):
    """Prints the message associated with a status code and returns the code"""
    config = Config(config_path)
    commands = {
        'temperature': set_temperature,
        'exposure': set_exposure,
        'status': status,
        'start': start,
        'stop': stop,
        'nvm': nvm,
        'init': initialize,
        'kill': shutdown,
    }

    if len(args) == 0 or (args[0] not in commands and args[0] != 'completion'):
        return print_usage(usage_prefix)

    if args[0] == 'completion':
        if 'start' in args[-2:]:
            print('continuous')
        elif 'temperature' in args[-2:]:
            print('warm')
        elif 'nvm' in args[-2:]:
            print('dump store erase')
        elif len(args) < 3:
            print(' '.join(commands))
        return 0

    try:
        ret = commands[args[0]](config, usage_prefix, args[1:])
    except KeyboardInterrupt:
        # ctrl-c terminates the running command
        ret = stop(config, args)

        # Report successful stop
        if ret == 0:
            ret = -100
    except Pyro4.errors.CommunicationError:
        ret = -101

    # Print message associated with error codes
    if ret not in [-1, 0]:
        print(CommandStatus.message(ret))

    return ret


def nvm(config, usage_prefix, args):
    if len(args) == 2:
        if args[0] == 'dump':
            with config.daemon.connect() as camd:
                return camd.dump_nvm(args[1])

    elif len(args) == 1:
        with config.daemon.connect(timeout=10) as camd:
            if args[0] == 'store':
                return camd.store_nvm()
            elif args[0] == 'erase':
                return camd.erase_nvm()

    print(f'usage: {usage_prefix} nvm (dump|store|erase)')
    return -1


def status(config, *_):
    """Reports the current camera status"""
    with config.daemon.connect() as camd:
        data = camd.report_status()

    state_desc = CameraStatus.label(data['state'], True)
    if data['state'] == CameraStatus.Acquiring:
        progress = f'{data["exposure_progress"]:.1f} / {data["exposure_time"]:.1f}s'
        state_desc += ' (' + TFmt.Bold + progress + TFmt.Clear + ')'

    # Camera is disabled
    print(f'   Camera is {state_desc}')
    if data['state'] != CameraStatus.Disabled:
        if data['state'] > CameraStatus.Idle:
            if data['sequence_frame_limit'] > 0:
                print(f'   Acquiring frame {TFmt.Bold}{data["sequence_frame_count"] + 1}' +
                      f' / {data["sequence_frame_limit"]}{TFmt.Clear}')
            else:
                print(f'   Acquiring {TFmt.Bold}UNTIL STOPPED{TFmt.Clear}')

        print(f'   Temperature is {TFmt.Bold}{data["sensor_temperature"]:.0f}\u00B0C{TFmt.Clear}' +
              ' (' + CoolerMode.label(data['cooler_mode'], True) + ')')

        if data['cooler_setpoint'] is not None:
            print(f'   Temperature set point is {TFmt.Bold}{data["cooler_setpoint"]:.0f}\u00B0C{TFmt.Clear}')

        print(f'   Exposure time is {TFmt.Bold}{data["exposure_time"]:.3f} s{TFmt.Clear}')
    return 0


def set_temperature(config, usage_prefix, args):
    """Set the camera temperature"""
    if len(args) == 1:
        if args[0] == 'warm':
            temp = None
        else:
            temp = int(args[0])
        with config.daemon.connect() as camd:
            return camd.set_target_temperature(temp)
    print(f'usage: {usage_prefix} temperature <degrees>')
    return -1


def set_exposure(config, usage_prefix, args):
    """Set the camera exposure time"""
    if len(args) == 1:
        exposure = float(args[0])
        with config.daemon.connect() as camd:
            return camd.set_exposure(exposure)
    print(f'usage: {usage_prefix} exposure <seconds>')
    return -1


def start(config, usage_prefix, args):
    """Starts an exposure sequence"""
    if len(args) == 1:
        try:
            count = 0 if args[0] == 'continuous' else int(args[0])
            if args[0] == 'continuous' or count > 0:
                with config.daemon.connect() as camd:
                    return camd.start_sequence(count)
        except Exception:
            print('error: invalid exposure count:', args[0])
            return -1
    print(f'usage: {usage_prefix} start (continuous|<count>)')
    return -1


def stop(config, *_):
    """Stops any active camera exposures"""
    with config.daemon.connect() as camd:
        return camd.stop_sequence()


def initialize(config, *_):
    """Enables the camera driver"""
    # Initialization can take more than 5 sec, so bump timeout to 10.
    with config.daemon.connect(10) as camd:
        return camd.initialize()


def shutdown(config, *_):
    """Disables the camera drivers"""
    with config.daemon.connect() as camd:
        return camd.shutdown()


def print_usage(usage_prefix):
    """Prints the utility help"""
    print(f'usage: {usage_prefix} <command> [<args>]')
    print()
    print('general commands:')
    print('   status       print a human-readable summary of the camera status')
    print('   exposure     set exposure time in seconds')
    print('   start        start an exposure sequence')
    print()
    print('engineering commands:')
    print('   init         connect to and initialize the camera')
    print('   temperature  set target temperature and enable cooling')
    print('   kill         disconnect from the camera')
    print()

    return 0
