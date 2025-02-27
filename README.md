# Pico-LiDAR
The goal of this project is to build a simple LiDAR sensor using hobbyist components
such as the Raspberry Pi Pico microcontroller, VL53L0X Time-of-Flight laser sensor
and the HC-SR04P ultrasonic sensor.

![](./docs/images/lidar_module.jpg)

## Bill of Materials (BOM)
<details>
<summary>Mechanical Parts (click to open)</summary>

- 608Z bearing
- generic 6 wire slip ring with 12.5mm diameter
- 7x M4x10 bolts (can be shorter)
- 6x M3x16 countersunk bolts (can be shorter) for mounting the dome top cover
  and the base bottom plate to their respective bodies.
- 7-8x M3x4 bolts for mounting the L298N and XL6009 to the base plate as well as the
  slipring to the axis mount.
  but you don't really need to use them all. The remaining screws are for the XL6009.
- 4x M2x6 bolts for mounting the perfboard
- 2-4x M2x4 bolts for mounting the Pi Pico
- 2x M4 nuts
  I used one with a 9mm contracted length (measured without the end rings) and a
  filament diameter of 0.5mm. Stronger springs are also possible but it should
  not be much longer.
- 5x M4x8x6mm threaded insert
- 6x M3x6x5mm threaded insert
- 1x rubber band
- (optional) 1x short tension spring.
- 5x cylindrical magnets 10x6mm. If you want to use different magnets feel
  free to modify the provided 3D files. 4 are used for speed measurement.
  The 5th is for detecting the 0 degree position
</details>

<details>
<summary>Electronic Parts (click to open)</summary>

- generic RF-320 DC Motor
- XL6009 DC-DC Buck Boost Converter module
- L298N DC Motor Driver module
- Raspberry Pi Pico
- 2x A3144 Hall Effect Sensor or equivalent
  Note: The A3144 is a 5V sensor but in my experience will also work
  when powered using 3.3V.
- VL53L0X Time-of-Flight sensor module
- HC-SR04P Ultrasonic Sensor module (The P version can be powered from 3.3V)
- 6pin 2.54mm female socket connector for VL53L0X sensor
- 4pin 2.54mm female socket connector for HC-SR04P sensor
- 1 push button for manually starting the motor
- 1 push button for resetting the MCU (optional)
- 1 LED for indicating motor state and the corresponding current limiting resistor
- (Optional) 20pin IDC connector for JLink. Used for development purposes.
- 1x 70x30mm Perfboard. Used for mounting interfaces and power connectors.
  Can be slightly larger and can be cut up from a larger board as long
  as the holes for mounting are present.
- (Optional) ESP01 or other ESP8266-based controller for wireless connectivity
  using esp-link.
</details>

## Assembly instruction
Refer to the [Assembly instructions document](./docs/assembly.md)

## Setup
### Flashing using J-Link (SWD Mode)
**By default the project is configured to use a J-Link Debug Probe in SWD mode.
To use a Pico Debug Probe instead, switch out any reference to `jlink` in the `platformio.ini`
file for `raspberrypi-swd`. Using USB for flashing is not recommended since the USB port
will be inaccessible inside the case therefore preventing flashing new firmware later.**

Wire up the J-Link probe according to the following table.
| J-Link Pin | J-Link Pin Name | Pico Pin | Pico Pin Name  |
| ---------- | --------------- | -------- | -------------- |
| 1          | VTref           | 37       | 3V3 (OUT)      |
| 4          | GND             | 38       | GND            |
| 5          | Tx (out)        | 2        | GP1 / UART0_RX |
| 7          | SWDIO           |          | SWDIO          |
| 9          | SWCLK           |          | SWCLK          |
| 15         | nRESET          | 30       | RUN            |
| 17         | Rx (in)         | 1        | GP0 / UART0_TX |
| 19         | 5V-Supply       | 39       | VSYS           |

**Warning: The 5V supply of the J-Link is insufficient to power the power the motor
and the pico. Connect a different power supply for the motor or the probe
will be unable to flash.**

**J-Link configuration:**
- The J-Link UART port has to be enable using the J-Link configurator utility
  if you have not done so already.
- The J-Link 5V-Supply pin is controlled using the J-Link commander utility.
  Use `power on` to turn it on or `power on perm` to make it stay on even after
  reconnecting the probe.

After setting up the J-Link you can flash the project using platformio.

### Installing dependencies for python script
To install the dependencies in a Python venv use the following commands
1. Create virtual environment
   > python -m venv .venv
2. Activate venv<br>
    Run the activation script in `.venv/Scripts/`corresponding to your platform
    from your current terminal. On windows that is either `activate.bat` or
    `Activate.ps1` depending on whether you use the command line or powershell.
    On linux use `activate` without any file extension.
3. Install dependencies from requirements.txt
   > pip install -r interface-script/requirements.txt

### Connections
Commands and communication from and to the LiDAR module are done using UART0
serial port of the pico, for which GP0 is the Tx pin and GP1 is Rx.

**Option 1: Connecting using USB serial adapter**

You can connect directly to the LiDAR module by using a USB serial adapter
connected to the aforementioned Rx and Tx pins.
The LiDAR uses a serial port configuration of 115200 baud 8N1.

The command for the interface script in that case would be.
> python interface-script/main.py -p \<serial-port>

Optionally you can specify the baudrate using `-b` but the default is set to
115200.

**Option 2: Wireless connection using esp-link**

When connecting to the device, either a USB serial adapter can be used, or an
ESP8266 module which was flashed using
[esp-link](https://github.com/jeelabs/esp-link)

The esp-link is again connected to the Rx and Tx pins using 115200 baud 8N1.
To connect the interface script to the esp-link use the `-d` parameter to specify
the devices hostname or ip-address.
> python interface-script/main.py -d \<hostname>

## Usage
### args file
All cli arguments you can pass to the interface script can also be written in a
plain text file to be passed to the program instead by using the `-f` parameter.

This is an example of what the content of this file could look like:
```
-p COM7
-r
-a 5000
```
The command to use this file would be
> python interface-script/main.py -f \<path to file>

## Attributions for CAD models
Unfortunately I did not keep track of the origin of 3D CAD models used
while building this project. If you think your model is being used and don't
want it to be, please open an issue and I will see about removing it.