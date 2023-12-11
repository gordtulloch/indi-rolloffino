# indi-rolloffino GT
INDI observatory rolloff roof driver to work with a Arduino controller

Forked from https://github.com/wotalota/indi-rolloffino with many thanks! Wotalota's code is a good general purpose starting point whereas mine is for a specific configuration:

- 8 port relay module
- Detection of RA and DEC park status of the telescope to ensure they are ON before a Close is allowed


## Changes from the original code:

### C++ code
- Changed all references to Lock and Aux to RA and DEC to support RA and DEC sensors to determine lock position
- Introduced code to prohibit a roof close unless the RA and DEC switches were ON to prevent the roof closing on the telescope
- Added error messages where a roof close is attempted while the RA and DEC switches are OFF

### INO Code
- Changed all references to Lock and Aux to RA and DEC to support RA and DEC sensors to determine lock position
- Introduced code to prohibit a roof close unless the RA and DEC switches were ON to prevent the roof closing on the telescope
- Lock and Aux relays are changed to Aux1 and Aux2 to support use for misc functions like fans, lights, etc.
- Added Aux3,Aux4,Aux5,Aux6 since I have 8 relays available and added support for on/off and status lights in the driver screen

### Application notes
I use my Aux ports for:

- Aux1	Dew heater on allskycam
- Aux2	Red lights
- Aux3	White lights
- Aux4	S camera
- Aux5	N camera
- Aux6	Future

### Status: still working on initial mods to the C++ so not currently checked in, INO is nearly done
