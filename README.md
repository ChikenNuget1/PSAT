# PSAT Team LEO
This is the official firmware repository for Team LEO PSAT.
## Installation
1. Install PlatformIO Extension on VSCode
```bash
git pull origin
```
3. Connect Device to USB
4. Build Code
5. Upload Code
6. Check Serial Monitor

## Troubleshooting Guide
```bash
Error: Please specify upload_port for environment or use global --upload-port option. For some development platforms it can be a USB flash drive (i.e. /media/<user>/<device name>) *** [upload] Explicit exit, status 1
```
Or any upload error
Usually means that the specific port on platformio.ini is incorrect.
1. Open Device Manager -> Ports -> Find the COM port that the device uses
2. Change upload_port and monitor_port in platformio.ini
