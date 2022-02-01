Device server for PI GCS2 cotor controller. Uses PI GCS library and pipython to communicate with controller via TCP/IP.

## Requirements
* `libpi_pi_gcs2` library (from software CD or downloaded from PI website; PI provides download links upon request)
* [pipython](https://pypi.org/project/PIPython/)

## Classes
### PIGCSController
Device server for the GCS controller. Get list of connected axes from `get_axis_names` command
Properties:
* controller IP address
* controller port (default: 50000)

### PIGCSAxis
Device server for a single axis connected to GCS controller.
Properties:
* tango FQDN of controller
* axis name (case-sensitive!)

## Authors
M. Schneider, MBI Berlin
