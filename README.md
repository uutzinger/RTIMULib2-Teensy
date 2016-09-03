# RTIMULib2-Teensy - a versatile 9-dof, 10-dof and 11-dof IMU library for the Teensy3.1 & 3.2

RTIMULib2-Teensy is the simplest way to connect a 9,10,11-dof IMU to a Teensy3.1 or 3.2 and obtain fully fused quaternion or Euler angle pose data. It will also read pressure sensors and humidity sensors. 

This fork of the RTIMULib2-Teensy includes drivers for underwater pressure sensor, humidity sensors, temperature calibration option, cross axis corrections for acceleration, option for runtime accelerometer calibration, storing more calibration data in the EEPROM, extended Arduino Sketch to verify calibration and visualize performance and Processing Sketch that visually displays the pose. Changes were made to the gyro bias calibration to better reject values.

*** If the fusion algorithm uses magnetometer data, it is critical that it is calibrated. Noise in the pose computation is usually due to magnetometer. When running this software on a robot it is best to turn off the magnetometer input into the fusion algorithm before motors are energized. ***

*** There is a runtime gyroscope bias computation which is engaged if the motion detection algorithm senses that the IMU is not moving. The motion detection algorithm is not perfect and if the user has a priori knowledge about the motion status, the gyroscope bias computation should only be engaged when the IMU is not moving. ***

*** ToDo: Magnetic distortion detection is partially implemented. a) Earth's magnetic field strength does not change based on pose of sensor. If field strength changes during motion, a motor or field anomaly is present. b) if the compass based heading differs from the fusion computed yaw, there is an anomaly. The fusion algorithm should automatically turn off magnetometer input if there is an anomaly. 

## Features

RTIMULib2-Teensy currently supports the following IMUs:

* InvenSense MPU-6050 plus HMC5883 magnetometer on MPU-6050's aux bus (handled by the MPU-9150 driver).
* InvenSense MPU-6050 gyros + acclerometers. Treated as MPU-9150 without magnetometers.
* InvenSense MPU-9150 single chip IMU.
* InvenSense MPU-9250 single chip IMU (I2C and SPI)
* InvenSense MPU-9255 single chip IMU (I2C and SPI)
* STM LSM9DS0 single chip IMU
* L3GD20H + LSM303D (optionally with the LPS25H) as used on the Pololu AltIMU-10 v4.
* L3GD20 + LSM303DLHC as used on the Adafruit 9-dof (older version with GD20 gyro) IMU. 
* L3GD20H + LSM303DLHC (optionally with BMP180) as used on the new Adafruit 10-dof IMU.
* Bosch BMX055 (although magnetometer support is experimental currently).
* Bosch BNO055 with onchip fusion.
9250 and 9150 are set to provide temperature readings and use FIFO for all sensor readings.

Pressure/temperature sensing is supported for the following pressure sensors:

* BMP180
* LPS25H
* MS5611
* MS5637
* MS5803 (10atm underwater)
* MS5837 (30atm underwater)

Humidity/temperature sensing is supported for the following humidity sensors:

* HTS221
* HTU21D

The humidity infrastructure and HTS221 support was generously supplied by XECDesign. It follows the model used by the pressure infrastructure - see RTIMULibDrive11 for an example of how to use this.

Note that currently only pressure sensors connected via I2C are supported.  Also, an MS5637 sensor will be auto-detected as an MS5611. To get the correct processing for the MS5637, edit the RTIMULib.ini file and set PressureType=5.

By default, RTIMULib2-Teensy will try to auto-discover IMUs and pressure sensors on I2C and SPI busses (only IMUs on the SPI bus). By default, the SPI IMU interface uses Teensy3.1 pin 9 as the chip select but this can be changed by editing libraries/RTIMULib/RTIMUSettings.h.

RTIMULib2-Teensy also supports multiple sensor integration fusion filters such as Kalman and AHRS filters. Note that the BNO055 always uses its onchip fusion results rather than any of the RTIMULib2-Teensy filters. Also, if performs its own magnetometer calibration so normal calibration data is not used.

If an SD card is available on the Teensy3.1, RTIMULib2-Teensy will use it for configuration data. This uses the SPI interface and pin 10 as select by default. This can be changed by editing libraries/RTIMULib/RTIMUSettings.h. Configuration will be stored in a file called RTIMULib.ini on the SD card. This can be edited by hand (on another machine with an SD card reader) if there is any need to change defaults or auto-detection settings. A simple sketch is provided that deletes this file if necessary - changing IMU type would be an example. The RTIMULib.ini file could be edited but it's quicker to just delete the ini file and start again if the IMU type is changed.

If no SD card is available, EEPROM is used just to save magnetometer and accelerometer max/min and gyroscope bias calibration data. If other settings need to be changed (such as sample rate), that should be done by changing values in the RTIMUSettings structure during setup phase of the Sketch program. 

The actual RTIMULib and support libraries are in the library directory. The other top level directories contain example sketches.


## Note about magnetometer (compass) calibration

For many IMUs, fused data may be completely unusable unless the magnetometers have been calibrated.

RTIMULib2-Teensy has two mechanisms that can be used to calibrate the magnetometers:

* Manual calibration. This is where TeensyMagCal has been used to set magnetometer calibration data. Once this is done, there should be no need to repeat calibration unless the magnetic environment changes. 

* Runtime calibration. This mechanism is used if there is no manual calibration data. The magnetometers will remain uncalibrated until a sufficient range of readings has been obtained in each of the axes. The code will continue to monitor magnetometer readings for new maxima and minima and update the calibration data as required. This data is not saved so the procedure will start from scratch if the code is restarted.

Also, if using a non-standard axis rotation, magnetometer calibration MUST be run AFTER changing the axis rotation.

## The Example Sketches

### Build and run

To build and run the example sketches, start the Teensyduino IDE and use File --> Preferences and then set the sketchbook location to:

	.../RTIMULib2-Teensy

where "..." represents the path to the RTIMULib2-Teensy directory. The directory is set up so that there's no need to copy the libraries into the main Arduino libraries directory although this can be done if desired.

### Cube_Display
This processing sketch displays the IMU as a box and shows numerical values in a window.
It responds to some user key strokes and for example turns compass on/off with M/m.
This visualization is useful to study how fast the pose follows motion and how well the motion detection algorithm works as one can observe very small motion using this program. If compass is used for fusion, after rigorous motion, there usually is delayed settling of the pose.

### BitBucketsIMU
This sketch implements IMU for FRC team 4183 to communicate with RoboRIO and transmit data over USB interface.
The sketch attempts reporting all data and has ability to turn on/off features of the fusion algorithm and gyroscope bias computation. It also estimates residual acceleration in earth coordinate system and computes velocity and attempts a position estimation. A velocity and acceleration bias is computed as when the sensor comes to a halt velocity should be zero.
The sketch accepts single character commands such as s/S for streaming, m/M to disable/enable compass, a/A accelerometer, g/G gyroscope, v/V to switch between readable text display.
In verbose mode, the max/min and bias calibration values are displayed. All vectors also display their length which is a performance indicator. For example if gyroscope is properly calibrated and the sensor is still the 9250 IMU has a residual of about 0.002 radians/sec. The compass rarely has a stability of better than 10% on the 9250. The accelerometer on the 9250 has about 10mg repeatability between on/off cycle but can be calibrated to about 5mg noise. Residual acceleration noise is about 0.02 m/s2 when the IMU is not moving. 
Three calibration procedures have been built in:
1) Gyroscope calibration can be run in the background and is constantly updated when the sensor is not moving. It is better to calibrate it when one knows there is no movement and then to turn off the gyroscope calibration.
2) Accelerometer calibration can be engaged when the sensor is held still. Ideally it is moved to a few different poses and when calibration is activated it will adjust values until 1g is read. Repeat the calibration a few times in all three directions in positive and negative direction. Axes do not need to be aligned perfectly.
3) Compass calibration. The new version of RTIMU has realtime compass calibration. This routine is activated and one will need to rotate the sensor to the maximum and minimum readings in all directions. Please be aware that magnetic field points into ground on northern hemisphere as well as towards north.

### TeensyMagCal
This sketch can be used to calibrate the magnetometers and should be run before trying to generate fused pose data. It also needs to be rerun at any time that the configuration is changed (such as different IMU or different IMU reference orientation). Load the sketch and all three axis of the IMU towards North and along the magnetic field lines, making sure all axes reach their minima and maxima. The display will stop updating when this occurs. Then, enter 's' followed by enter into the IDE serial monitor to save the data.
This calibration should be run in a distortion free environment.

### TeensyAccelCal
This sketch allows updating the max/min data of the calibration files and does not require the sensor to be aligned with one axis as it will proportionally change max & min values until the recorded acceleration is 1g.
Make sure the sensor is still, then enable calibration by entering 'A'. After about a second enter 'a' and flip the compass towards opposite direction and repeat the process. Repeat this process until the sensor shows 1g in all possible directions.
Enter 's' to save the data.
Do not run autocalibration or update max/min data when you move the sensor!

### TeensyGyroCal
This will run the sensor for a while and then save the gyro bias values in the EEPROM. Bias values are also updated during runtime when the sensor is not moving.

### TeensyIMU
TeensyIMU is the main demo sketch. It configures the IMU based on settings in RTIMUSettings.cpp. Change these to alter any of the parameters or edit the RTIMULib.ini file after auto-detection.

### TeensyIMU10/11
This is exactly the same as TeensyIMU except that it adds support for a pressure/humidity sensor. If a pressure sensor was found during auto-detection, this will be used by the sketch. If not pressure sensors were found, the sketch will report this and stop.

### TeensyAccel
This is similar to TeensyIMU except that it subtracts the rotated gravity vector from the accelerometer outputs in order to obtain the residual accelerations - i.e. those not attributable to gravity.

### TeensyDeleteEEPROM
A simple sketch that can be downloaded to clear mag calibration data from the EEPROM. This is useful if it is desired to revert to runtime magnetometer calibration.

### TeensyDeleteIni
A simple sketch that can be downloaded to delete the RTIMULib.ini file from the SD card.
