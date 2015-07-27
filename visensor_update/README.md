# README #

## Installation visensor_update ###

Check out the sensor library and this node to your catkin workspace:

```
cd your_catkin_workspace
git@github.com:catkin/catkin_simple.git
git clone https://github.com/ethz-asl/libvisensor.git
git clone https://github.com/skybotix/visensor_tools.git
```

Make sure that you installed all necessary ROS packages

```
sudo apt-get install libssh2-1-dev libeigen3-dev libboost-dev ros-indigo-cmake-modules
```
Adjust the packet name to your ros version.

Build the package using catkin_make

```
catkin build
```


### Howto: Update the VI-Sensor###
Make sure that you are connected to the internet. Make sure that the sensor is connected and you know its IP. If the sensor is directly connected to the PC, its IP is 10.0.0.1 :

Run visensor_update tool (assuming that the sensor IP is 10.0.0.1 and the sensor has a ADIS16448 IMU)
```
rosrun visensor_update visensor_update 10.0.0.1 update
```

If the sensor has a ADIS16488 IMU (tactical grade), adjust the update command as follows:
```
rosrun visensor_update visensor_update 10.0.0.1 update 16488
```

If everything went smoothly, the sensor will reboot after the update and the terminal output is something like

```
After update:
Name				Version
-----------------------------------------
visensor-fpga-bitstream		0.0.8
visensor-kernel-modules		0.0.3
visensor-linux-embedded		0.1.1

Rebooting sensor...
```

For major release updates the update tool may ask you for different configuration settings as the Sensor ID. Please enter the corresponding questions which look like:
```
Downloading visensor-fpga-bitstream ...  done.
Downloading visensor-kernel-modules ...  done.
Downloading visensor-linux-embedded ...  done.

Load old format ... done.
Load new configuration in case the conversation is done multiple time... done.
Convert calibration to new format ... done.
The new configuration requested the Vi-Sensor ID.
Sensor ID (integer):
```