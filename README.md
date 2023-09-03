## Raptor camera daemon

`raptor_camd` interfaces with and wraps Raptor Ninox 1280 detectors and exposes them via Pyro.

`swir` is a commandline utility for controlling the cameras.

### Configuration

Configuration is read from json files that are installed by default to `/etc/camd`.
A configuration file is specified when launching the camera server, and the `swir` frontend will search for files matching the specified camera id when launched.

The configuration options are:
```python
{
  "daemon": "localhost_test", # Run the camera server as this daemon. Daemon types are registered in `rockit.common.daemons`.
  "pipeline_daemon": "localhost_test2", # The daemon that should be notified to hand over newly saved frames for processing.
  "pipeline_handover_timeout": 10, # The maximum amount of time to wait for the pipeline daemon to accept a newly saved frame. The exposure sequence is aborted if this is exceeded.
  "log_name": "raptor_camd@test", # The name to use when writing messages to the observatory log.
  "control_machines": ["LocalHost"], # Machine names that are allowed to control (rather than just query) state. Machine names are registered in `rockit.common.IP`.
  "cooler_setpoint": -15, # Default temperature for the SWIR sensor.
  "cooler_update_delay": 5, # Amount of time in seconds to wait between querying the camera temperature and cooling status.
  "worker_processes": 3, # Number of processes to use for generating fits images and saving temporary images to disk.
  "framebuffer_bytes": 616512000, # Amount of shared memory to reserve for transferring frames between the camera and output processes (should be an integer multiple of frame size).
  "header_card_capacity": 144, # Pad the fits header with blank space to fit at least this many cards without reallocation.
  "camera_id": "TEST", # Value to use for the CAMERA fits header keyword.
  "output_path": "/var/tmp/", # Path to save temporary output frames before they are handed to the pipeline daemon. This should match the pipeline incoming_data_path setting.
  "output_prefix": "test", # Filename prefix to use for temporary output frames.
  "expcount_path": "/var/tmp/test-counter.json" # Path to the json file that is used to track the continuous frame number.
}
```

### Initial Installation

The first step is to download and install the EPIX Linux SDK from their website.

The automated packaging scripts will push 4 RPM packages to the observatory package repository:

| Package                         | Description                                                                        |
|---------------------------------|------------------------------------------------------------------------------------|
| rockit-camera-raptor-server     | Contains the `raptor_camd` server and systemd service files for the camera server. |
| rockit-camera-raptor-client     | Contains the `swir` commandline utility for controlling the camera server.         |
| rockit-camera-raptor-data-clasp | Contains the json configuration files for the CLASP instrument.                    |
| python3-rockit-camera-raptor    | Contains the python module with shared code.                                       |

After installing packages, the systemd service should be enabled:
```
sudo systemctl enable --now raptor_camd.service@<config>
```

where `config` is the name of the json file for the appropriate camera.

Now open a port in the firewall so the TCS and dashboard machines can communicate with the camera server:
```
sudo firewall-cmd --zone=public --add-port=<port>/tcp --permanent
sudo firewall-cmd --reload
```

where `port` is the port defined in `rockit.common.daemons` for the daemon specified in the camera config.

### Upgrading Installation

New RPM packages are automatically created and pushed to the package repository for each push to the `master` branch.
These can be upgraded locally using the standard system update procedure:
```
sudo yum clean expire-cache
sudo yum update
```

The daemon should then be restarted to use the newly installed code:
```
sudo systemctl restart raptor_camd@<config>
```

### Testing Locally

The camera server and client can be run directly from a git clone:
```
./raptor_camd test.json
CAMD_CONFIG_ROOT=. ./swir test status
```
