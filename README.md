# Pico-LiDAR
The goal of this project is to build a simple LiDAR sensor using hobbyist components
such as the Raspberry Pi Pico microcontroller, VL53L0X Time-of-Flight laser sensor
and the the HC-SR04P ultrasonic sensor.

## Bill of Materials (BOM)
<details>
<summary>Mechanical Parts (click to open)</summary>

- 608Z bearing
- generic 6 wire slip ring with 12.5mm diameter
- 3x M3x8 bolts for mounting the slip ring
- 7x M4x10 bolts (can be shorter)
- 6x M3x16 countersunk bolts (can be shorter) for mounting the dome top cover
  and the base bottom plate to their respective bodies.
- 4-6x M3x4 bolts for mounting the L298N to the base plate. There are 4 screw holes
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
ToDo
### Required Tools
- Soldering Iron
- Heatgun
- Screwdrivers

### Mounting the motor
Depending on which DC motor you get the mounting holes on the corresponding
3D printed bracket may not line up. In that case you will need to edit the model
yourself or drill fitting holes in the existing part.

The 3D model is located in the [model](./model/) directory as a step file.

## Setup
### Installing dependencies

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