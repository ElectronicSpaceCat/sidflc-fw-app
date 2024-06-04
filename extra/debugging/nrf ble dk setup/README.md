# nRF Connect Bluetooth LowEnergy Setup

The device can be connected to an nRF development kit (DK) using the nRF Connect BLE software.

The following guide gives installation instructions as well as usage:

https://infocenter.nordicsemi.com/pdf/nRFConnect_BLE_User_Guide_v4.0.0.pdf

Ubuntu: Fix nRF BLE stand-alone app if getting blank screen after device connection:

Create a nRF_ble.desktop file on /Desktop

sudo nano nRF_ble.desktop

Copy the following:
[Desktop Entry]
Version=1.0
Type=Application
Terminal=false
Exec=bash -c 'LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6 ~/opt/nrfconnect-bluetooth-low-energy/nrfconnect-bluetooth-low-energy-4.0.4-x86_64.AppImage'
Name=nRF_BLE
Name[en_US]=nRF_BLE
Icon=blueman-active
Icon[en_US]=blueman-active
Categories=
Hidden=false
Comment[en_US]=nRF Bluetooth Low Energy

