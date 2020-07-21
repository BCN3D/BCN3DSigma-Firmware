# BCN3D Sigma Firmware based on Marlin
![image](https://user-images.githubusercontent.com/17469944/45097127-679ac200-b122-11e8-9a7a-f060e23c3201.jpg)

This is the repository that contains the firmware for the BCN3D Sigma 3D Printer. It's based on the well-known Marlin but with some modifications.

If you want to download the latest stable firmware version, go to [Releases page][7] and download the needed files. In the releases page you will find the *Source Code* and the SD Files needed for the LCD Display.

Please refer to the [wiki][8] in order to get instructions on how to upgrade the firmware and load new LCD SD files to the display.

> ## IMPORTANT
> This repo contains the firmware and the files for the LCD Touchscreen display (for now). When a new release is done, normally is needed updating the SD files of the display. That way we can update menus, graphics and functionalities. You can find more information on how to update the [LCD SD files here][5].

## Features

- Dual independent X extruders.
- Duplication & Mirror modes.
- Autolevel, autocalibration of XYZ axis.
- LCD Touchscreen support.
- Shims installation assisted.
- Load/Unload filament.
- Refined firmware for better printing experience.
- Filament Runout Sensor.

This firmware was first developed by [Jordi Calduch][1] then was [Xavier Gómez][2] and now the current developer is [Alejandro Garcia][6] at [BCN3D Technologies][3]. This firmware is a rework of the popular [Marlin][4].

The code is currently in development, trying to improve functionalities.


## Issues, bugs and suggestions

Your feedback is very important to us as it helps us improve even faster. Therefore we think github is the way to go. Feel free to make an [issue](https://github.com/BCN3D/BCN3DSigma-Firmware/issues).

In order to be more efficient with the feedback, we recommend to follow some guidelines:

+ First of all. Search for related issues.
+ Detail the firmware version you're running. It is displayed during start-up at the right bottom corner of the splash screen.
+ Try to explain us how to reproduce the error or bug, that way we can test it properly.
+ On the title, indicate the [label](https://github.com/BCN3D/BCN3DSigma-Firmware/labels) of the issue. For example > `[enhancement]: Some changes..`

## Development
For advanced users, it's possible to contribute in firmware development. We explain everything in the [wiki][9]. So if you are interested in contributing, folow the instructions [there][9]. 

The `master` branch is stable  and it's currently on version `2.0.8`.





[1]:https://github.com/dryrain
[2]:https://github.com/xawox
[3]:https://www.bcn3dtechnologies.com/
[4]:https://github.com/MarlinFirmware/Marlin
[5]:https://support.bcn3d.com/knowledge/update-micro-sd-files-lcd-bcn3d-sigma
[6]:https://github.com/AlejandroGarcia92
[7]:https://github.com/BCN3D/BCN3DSigma-Firmware/releases
[8]:https://github.com/BCN3D/BCN3D-Cura-Windows/wiki
[9]:https://github.com/BCN3D/BCN3DSigma-Firmware/wiki
