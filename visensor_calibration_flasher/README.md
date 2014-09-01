# README #

### HowTo build visensor_calibration_flasher ###

Clone repository into your ROS catkin workspace

```
git clone https://github.com/skybotix/visensor_tools.git

```
Build your catkin workspace
```
cd <your_caktin_workspace>
catkin_make
```

### HowTo flash a calibration ###
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
To test the program, we attached an example camchain file to this repo.

Send calibration to VI-Sensor:
```
rosrun vi_calibration_flasher vi_calibration_flasher

```
