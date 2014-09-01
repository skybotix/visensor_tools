# README #


### HowTo flash a calibration ###
clear other calibrations, jsut to be sure:

```
#!bash

rosparam delete /vi_calibration_flasher

```
load calibration file:

```
#!bash

rosparam load ~/camchain-imucam-run1.yaml

```
send calibration to vi-sensor:
```
#!bash
rosrun vi_calibration_flasher vi_calibration_flasher

```