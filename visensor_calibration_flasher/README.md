# README #

### Howto: Build visensor_calibration_flasher ###

Clone repository into your ROS catkin workspace
```
cd <your_caktin_workspace>/src
git clone https://github.com/skybotix/visensor_tools.git
```
Build your catkin workspace
```
cd <your_caktin_workspace>
catkin_make
```

### Howto: Flash a calibration file###

To test the program, we attached an example camchain file to this repo. You can generate your own camera-imu calibration using the framework [Kalibr](https://github.com/ethz-asl/kalibr).

Once the calibration using Kalibr is done, you can update the camchain-imucam-example.yaml file using the new values.

You can now upload the new calibration to the VI-Sensor

**Please note that you will overwrite the factory calibration**:
```
roslaunch visensor_calibration_flasher flash_sensor.launch
```
If you get a *Calibration upload succeeded!*, everything went smoothly. You can test the new calibration by starting the block-matcher launch file of the visensor_node. If you managed to flash your sensor incidentally, but still want the factory calibration, contact support@skybotix.com .
```
roslaunch visensor_node dense.launch
```
If everything went smoothly, the rectified and disparity image should look fine (Rectified image: straight lines should be straight, also towards the image corners. Disparity image: If scene is sufficiently textured, disparity map should be dense and fairly outlier-free).

