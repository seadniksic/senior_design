# Datasheet
https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

# Schematics
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/downloads

# Breakout Board Pinout
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts


# Config
- Auto boots to normal mode so no config there needed.
- default mode after power on is CONFIGMODE
- modes
  - get all the outputs raw and unfused, use mode AMG
  - NDOF_FMC_OFF uses accel mag and gyro and then provides absolute orientation
    - so does NDOF mode but idk what the difference is.
    - NDOF uses more power. FMC = fast magnetometer calibration
  - operating mode set by writing to OPR_MODE reg, see table 3-5
  - TAKES 7ms to switch from configmode to any operation mode
  - vice verse, its 19ms
- fusion modes
  - absolute with respect to earth's magnetic field
  - calculates direction  of the north pole. 
  - In non-absolute or relative orientation modes, the heading of the sensor can vary depending on how the sensor is placed initially.
- can remap axes depending on mounting and can also remap the sign of the axis too. like switch the direction. 
- if running in fusion mode, NEED to go back to configuration mode to configure it. 
- you can select your units!!


- M4G to start. no magnetometer calibration is needed.


# NOTES ON READ/WRITE
Any reserved bit is ignored when it is written and
no specific value is guaranteed when read. It is recommended not to use registers at all
which are completely marked as ‘reserved’. Furthermore it is recommended to mask out
(logical and with zero) reserved bits of registers which are partially marked as reserved.