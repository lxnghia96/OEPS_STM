# Oeps
A USB-controlled potentiostat/galvanostat for thin-film battery characterization based on tdstatv3
https://github.com/thomasdob/tdstatv3/


## Introduction
This repository contains all the necessary design files to build your own USB-controlled potentiostat/galvanostat.
Porting from Pic 16F1459 to STM32F072

### Directories


### Files
* `README.md`: This file.

## USB access
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="a0a0", ATTRS{idProduct}=="0002", GROUP="plugdev", MODE="0666"
```
This assumes that the current user is a member of the `plugdev` group, and that the default USB Vendor and Product ID's
as coded in the microcontroller firmware are used; if not, these values need to be adjusted.

## Credits
* NghiaLuu
* email: lxnghia96@gmail.com


