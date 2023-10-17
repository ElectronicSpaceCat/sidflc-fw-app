#!/bin/bash

# Place this file at the top project directory

# ============================================================
# This script generates the bootloader settings and then
# merges the bootloader + bootloader settings + soft device + application
# into one hex file for production programming.
# ============================================================
# NOTES:
#
# nrf-command-line-tools required to merge hex files, can be downloaded at:
# https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download
#
# Install nrf tools by running the commands:
#     cd ./nrfutil
#     nrfutil install nrf5sdk-tools
#
# Family setting  | nRF devices
# ---------------------------------------------------------
# NRF51           | nRF51xxx
# NRF52           | nRF52832,           nRF52833
# NRF52QFAB       | nRF52832-QFAB,      nRF52820
# NRF52810        | nRF52810,           nRF52811, nRF52805
# NRF52840        | nRF52840
#
# SDK version     BL settings version
# <=12.0          1
# >=15.3.0        2
#
# bootloader-version is user defined
# ============================================================
#
# Version types:
# --application-version 1
# --application-version-string "1.0.0"  <-- currently used for this project

# Proj Name Prefix
PROJ_NAME="sidflc"

# Change path below to point to the nrfutil executable
NRF_UTIL="../../nrfutil"

# Change path below to point to the nRF softdevice (SD) hex file
NRF_SD="../../nRF5_SDK_17.1.0_ddde560/components/softdevice/s112/hex/s112_nrf52_7.2.0_softdevice.hex"

# Applicaiton location
FW="../../${PROJ_NAME}-fw-app/build/${PROJ_NAME}_fw_app.hex"

# Bootloader location
BL="../../${PROJ_NAME}-fw-bootloader/build/${PROJ_NAME}_fw_bootloader.hex"

# Output file name
OUTPUT_NAME="${PROJ_NAME}_fw_app_prod"

VERSION="1.0.0"

$NRF_UTIL settings generate --family NRF52 --application ${FW} --application-version-string ${VERSION} --bootloader-version 0 --bl-settings-version 2 bl_setting.hex

mergehex --merge bl_setting.hex ${BL} ${NRF_SD} --output temp_merge.hex

mergehex --merge temp_merge.hex ${FW} --output "../../${PROJ_NAME}-fw-app/${OUTPUT_NAME}.hex"

rm bl_setting.hex
rm temp_merge.hex

