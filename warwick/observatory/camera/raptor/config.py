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

"""Helper function to validate and parse the json config file"""

# pylint: disable=too-many-instance-attributes

import json
from warwick.observatory.common import daemons, IP, validation

CONFIG_SCHEMA = {
    'type': 'object',
    'additionalProperties': False,
    'required': [
        'daemon', 'pipeline_daemon', 'pipeline_handover_timeout', 'log_name', 'control_machines',
        'camera_config_path', 'camera_model', 'camera_serial', 'camera_id', 'cooler_setpoint', 'cooler_update_delay',
        'worker_processes', 'framebuffer_bytes', 'header_card_capacity', 'output_path', 'output_prefix', 'expcount_path'
    ],
    'properties': {
        'daemon': {
            'type': 'string',
            'daemon_name': True
        },
        'pipeline_daemon': {
            'type': 'string',
            'daemon_name': True
        },
        'pipeline_handover_timeout': {
            'type': 'number',
            'min': 0
        },
        'log_name': {
            'type': 'string',
        },
        'control_machines': {
            'type': 'array',
            'items': {
                'type': 'string',
                'machine_name': True
            }
        },
        'camera_config_path': {
            'type': 'string'
        },
        'camera_model': {
            'type': 'string'
        },
        'camera_serial': {
            'type': 'integer'
        },
        'cooler_setpoint': {
            'type': 'number',
            'min': -50,
            'max': 30,
        },
        'cooler_update_delay': {
            'type': 'number',
            'min': 0
        },
        'worker_processes': {
            'type': 'integer',
            'min': 1
        },
        'framebuffer_bytes': {
            'type': 'integer',
            'min': 1280*800*2
        },
        'header_card_capacity': {
            'type': 'integer',
            'min': 0
        },
        'camera_id': {
            'type': 'string',
        },
        'output_path': {
            'type': 'string',
        },
        'output_prefix': {
            'type': 'string',
        },
        'expcount_path': {
            'type': 'string',
        }
    }
}


class Config:
    """Daemon configuration parsed from a json file"""
    def __init__(self, config_filename):
        # Will throw on file not found or invalid json
        with open(config_filename, 'r', encoding='utf-8') as config_file:
            config_json = json.load(config_file)

        # Will throw on schema violations
        validation.validate_config(config_json, CONFIG_SCHEMA, {
            'daemon_name': validation.daemon_name_validator,
            'machine_name': validation.machine_name_validator,
            'directory_path': validation.directory_path_validator,
        })

        self.daemon = getattr(daemons, config_json['daemon'])
        self.pipeline_daemon_name = config_json['pipeline_daemon']
        self.pipeline_handover_timeout = config_json['pipeline_handover_timeout']
        self.log_name = config_json['log_name']
        self.control_ips = [getattr(IP, machine) for machine in config_json['control_machines']]
        self.camera_config_path = config_json['camera_config_path']
        self.camera_model = config_json['camera_model']
        self.camera_serial = config_json['camera_serial']
        self.camera_id = config_json['camera_id']
        self.output_path = config_json['output_path']
        self.output_prefix = config_json['output_prefix']
        self.expcount_path = config_json['expcount_path']
        self.worker_processes = config_json['worker_processes']
        self.framebuffer_bytes = config_json['framebuffer_bytes']
        self.header_card_capacity = config_json['header_card_capacity']
        self.cooler_setpoint = config_json['cooler_setpoint']
        self.cooler_update_delay = config_json['cooler_update_delay']
