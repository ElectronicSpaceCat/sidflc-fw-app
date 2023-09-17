#!/bin/bash

# Install nrf tools by running the commands:
#     cd ./nrfutil
#     nrfutil install nrf5sdk-tools

# Version types:
# --application-version 1
# --application-version-string "1.0.0"    <-- currently used for this project

# Modify the path defines as needed

NRF_UTIL_LOC="../../.."
HEX_FILE="../../../sidflc-fw-app/build/sidflc_fw_app.hex"
OUTPUT_NAME="sidflc"
VERSION="1.0.1"

$NRF_UTIL_LOC/nrfutil pkg generate --hw-version 52 --application-version-string $VERSION --application $HEX_FILE --sd-req 0x103 --key-file private.key $OUTPUT_NAME.zip

echo $VERSION

# Create version file
echo $VERSION > version
