# indi-rolloffnano
INDI observatory rolloff roof driver to work with a Arduino nano controller

Forked from https://github.com/wotalota/indi-rolloffino with many thanks! Wotalota's code is a good general purpose starting point whereas mine is for a specific configuration:

- Arduino Nano
- A single relay which triggers an open/close of a roof (and no aborts)
- Detection of RA and DEC park status of the telescope to ensure they are ON before a Close is allowed

## Changes from the original code:

### C++ code
- Changed all references to Lock and Aux to RA and DEC to support RA and DEC sensors to determine lock position
- Introduced code to prohibit a roof close unless the RA and DEC switches were ON to prevent the roof closing on the telescope
- Added error messages where a roof close is attempted while the RA and DEC switches are OFF

### INO Code
- Changed all references to Lock and Aux to RA and DEC to support RA and DEC sensors to determine lock position
- Introduced code to prohibit a roof close unless the RA and DEC switches were ON to prevent the roof closing on the telescope

### Status: still working on initial mods to the C++ so not currently checked in, INO is nearly done
