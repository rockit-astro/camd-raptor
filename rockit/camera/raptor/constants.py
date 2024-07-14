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

"""Constants and status codes used by raptor-camd"""

# pylint: disable=too-few-public-methods
# pylint: disable=invalid-name


class CommandStatus:
    """Numeric return codes"""
    # General error codes
    Succeeded = 0
    Failed = 1
    Blocked = 2
    InvalidControlIP = 3

    CameraNotFound = 5

    # Command-specific codes
    CameraNotInitialized = 10
    CameraNotIdle = 11
    CameraNotUninitialized = 14
    CameraNotAcquiring = 15

    TemperatureOutsideLimits = 20

    _messages = {
        # General error codes
        1: 'error: command failed',
        2: 'error: another command is already running',
        3: 'error: command not accepted from this IP',
        5: 'error: camera hardware not found',

        # Command-specific codes
        10: 'error: camera has not been initialized',
        11: 'error: camera is not idle',
        14: 'error: camera has already been initialized',
        15: 'error: camera is not acquiring',

        20: 'error: requested temperature is outside the supported limits',

        -100: 'error: terminated by user',
        -101: 'error: unable to communicate with camera daemon',
    }

    @classmethod
    def message(cls, error_code):
        """Returns a human readable string describing an error code"""
        if error_code in cls._messages:
            return cls._messages[error_code]
        return f'error: Unknown error code {error_code}'


class CameraStatus:
    """Status of the camera hardware"""
    # Note that the Reading status is assumed at status-query time
    # and is never assigned to CameraDaemon._status
    Disabled, Initializing, Idle, Waiting, Acquiring, Reading, Aborting = range(7)

    _labels = {
        0: 'OFFLINE',
        1: 'INITIALIZING',
        2: 'IDLE',
        3: 'WAITING',
        4: 'EXPOSING',
        5: 'READING',
        6: 'ABORTING'
    }

    _colors = {
        0: 'red',
        1: 'red',
        2: 'default',
        3: 'yellow',
        4: 'green',
        5: 'yellow',
        6: 'red'
    }

    @classmethod
    def label(cls, status, formatting=False):
        """
        Returns a human readable string describing a status
        Set formatting=true to enable terminal formatting characters
        """
        if formatting:
            if status in cls._labels and status in cls._colors:
                return f'[b][{cls._colors[status]}]{cls._labels[status]}[/{cls._colors[status]}][/b]'
            return '[b][red]UNKNOWN[/red][/b]'

        if status in cls._labels:
            return cls._labels[status]
        return 'UNKNOWN'


class CoolerMode:
    """Camera temperature control mode"""
    Unknown, Off, Locking, Locked = range(4)

    _labels = {
        0: 'UNKNOWN',
        1: 'OFF',
        2: 'LOCKING',
        3: 'LOCKED',
    }

    _colors = {
        0: 'red',
        1: 'red',
        2: 'yellow',
        3: 'green'
    }

    @classmethod
    def label(cls, status, formatting=False):
        """
        Returns a human readable string describing a status
        Set formatting=true to enable terminal formatting characters
        """
        if formatting:
            if status in cls._labels and status in cls._colors:
                return f'[b][{cls._colors[status]}]{cls._labels[status]}[/{cls._colors[status]}][/b]'
            return '[b][red]UNKNOWN[/red][/b]'

        if status in cls._labels:
            return cls._labels[status]
        return 'UNKNOWN'
