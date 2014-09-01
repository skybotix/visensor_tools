# README #

This repo contains some tools to interface the VI-Sensor:

* **visensor_calibration_flasher**: Uploads VI-Sensor calibration on sensor firmware. The device itself is factory calibrated. However, if you changed the camera/IMU configuration, you can calibrate it using the [Kalibr](https://github.com/ethz-asl/kalibr/) framework. The resulting camchain file can be uploaded to sensor using this script.

* **visensor_update**: Updates your VI-Sensor to newest firmware version. Checks online if your sensor is up to date and updates it if necessary. 
