# Pico-LiDAR
The goal of this project is to build a simple LiDAR sensor using hobbyist components
such as the Raspberry Pi Pico microcontroller, VL53L0X Time-of-Flight laser sensor
and the the HC-SR04 ultrasonic sensor.

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
- HC-SR04 Ultrasonic Sensor module
- 6pin 2.54mm female socket connector for VL53L0X sensor
- 4pin 2.54mm female socket connector for HC-SR04 sensor
- 1 push button for manually starting the motor
- 1 push button for resetting the MCU (optional)
- 1 LED for indicating motor state and the corresponding current limiting resistor
- (Optional) 20pin IDC connector for JLink. Used for development purposes.
- 1x 70x30mm Perfboard. Used for mounting interfaces and power connectors.
  Can be slightly larger and can be cut up from a larger board as long
  as the holes for mounting are present.
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

3D files for editing will be provided.

## Attributions for CAD models
Unfortunately I did not keep track of the origin of 3D CAD models used
while building this project. If you think your model is being used and don't
want it to be, please open an issue and I will see about removing it.