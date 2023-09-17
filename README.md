# Firmware

Suggested project folder structure:
```
sidflc/
├── gcc-arm-none-eabi
├── nRF_SDK
├── nrfutil
├── sidflc-fw-app
├── sidflc-fw-bootloader
└── prod_hex_merge.bat/sh
```

The firmware prod_sidflc_app.hex is a merge of the app + softdevice + bootloader + bootloader settings. The prod_hex_merge.bat will do this but the nrfutil.exe is required to run it. The path to the utility in the script should point to its location on your system.

The nrfutil can be found here:  
https://www.nordicsemi.com/Products/Development-tools/nrf-util

The nRF5 SDK is required to build the project:  
https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/sdks/nrf5/binaries/nrf5_sdk_17.1.0_ddde560.zip  

ARM toolchain Version 10.3-2021.10 (the latest verison would likely work too)  
https://developer.arm.com/downloads/-/gnu-rm

nrf-command-line-tools required to merge hex files, downloaded at:  
https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download

nrf5sdk-tools required to create the bootloader settings and can be installed by the following commands: 
```
cd ./nrfutil
nrfutil install nrf5sdk-tools
```
Segger J-Link is recommended for flashing the firmware over the device's SWD on the main pcb.

## TODO
* Instructions on flashing firmware via segger
