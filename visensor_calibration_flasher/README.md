# README #

### Howto: Build visensor_calibration_flasher ###

Clone repository into your ROS catkin workspace
```
git clone https://github.com/skybotix/visensor_tools.git
```
Build your catkin workspace
```
cd <your_caktin_workspace>
catkin_make
```

### Howto: Flash a calibration file###
Start roscore, if not already started
```
roscore
```
Clear other calibrations, just to be sure:
```
rosparam delete /vi_calibration_flasher
```
Load calibration file:
```

rosparam load <path_to_your_camchain_yaml_file>
```
To test the program, we attached an example camchain file to this repo. You can generate your own calibration using the framework [Kalibr](https://github.com/ethz-asl/kalibr).

Send calibration to VI-Sensor:
```
rosrun vi_calibration_flasher vi_calibration_flasher
```
If you get a **Calibration upload succeeded!**, everything went smoothly. You can test the new calibration by starting the block-matcher launch file of the visensor_node. 
```
roslaunch visensor_node dense.launch
```
If everything went smoothly, the rectified and disparity image should look fine (Rectified image: straight lines should be straight, also towards the image corners. Disparity image: If scene is sufficiently textured, disparity map should be dense and fairly outlier-free)

