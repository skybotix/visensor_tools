# README #

### Howto: Build visensor_update ###

Clone repository onto your harddrive
```
git clone https://github.com/skybotix/visensor_tools.git
```
Make sure that you have all necessary packages installed in order to compile update package
```
sudo apt-get install libssh2-1-dev libboost-regex-dev
```
Go to visensor_update directory, create build directory and compile it
```
cd visensor_tools/visensor_update
mkdir build
cd build
cmake ..
make
```

### Howto: Update the VI-Sensor###
Make sure that you are connected to the internet. Make sure that the sensor is connected and you know its IP. If the sensor is directly connected to the PC, its IP is 10.0.0.1 :

Run visensor_update tool (assuming that the sensor IP is 10.0.0.1
```
cd visensor_tools/visensor_update/bin
./visensor_update 10.0.0.1 update
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
