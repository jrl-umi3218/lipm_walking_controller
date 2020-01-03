# LIPM Walking Controller

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/lipm_walking_controller/workflows/CI/badge.svg?branch=topic/ci)](https://github.com/jrl-umi3218/lipm_walking_controller/actions?query=workflow%3A%22CI%22)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/index.html)

[![Stair climbing by the HRP-4 humanoid robot](https://scaron.info/images/stair-climbing.jpg)](https://www.youtube.com/watch?v=vFCFKAunsYM&t=22)

Source code of the walking and stair climbing controller used in the experiments of [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document), as well as in an industrial demonstrator at the [Airbus Saint-Nazaire factory](https://cordis.europa.eu/project/rcn/194280/brief/en?WT.mc_id=exp).

## Getting started

The easiest way to get started with the controller is to run its Docker image from an Ubuntu Linux distribution:

```
xhost +local:docker
docker run -it --rm --user ayumi -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    stephanecaron/lipm_walking_controller \
    lipm_walking --floor
```

See the [documentation](http://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/docker.html) for more usage instructions.

## Installation

The controller has been tested on Ubuntu 14.04, Ubuntu 16.04 and Ubuntu 18.04. See the instructions to [build and install](http://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/build.html) in the documentation.

## Usage

If it is not there already, enable the controller in your mc\_rtc configuration:
```json
{
  "MainRobot": "JVRC1",
  "Enabled": ["LIPMWalking"]
}
```
Launch RViz for the JVRC-1 model by:
```sh
roslaunch mc_rtc_ticker display.launch
```
Finally, start the controller from your mc\_rtc interface. Here is the example
of the [Choreonoid](https://choreonoid.org/en/) project installed from
[mc\_openrtm](https://github.com/jrl-umi3218/mc_openrtm) in the
Docker image:
```sh
cd /usr/share/hrpsys/samples/JVRC1
choreonoid --start-simulation sim_mc.cnoid
```
You should end up with the following windows:
![Choreonoid and RViz GUI of the controller](https://user-images.githubusercontent.com/1189580/64157945-ead71c80-ce37-11e9-9081-7936702c5fbc.png)
See the [Graphical user interface](https://github.com/stephane-caron/lipm_walking_controller/wiki/Graphical-user-interface) page of the
wiki for further instructions on how to use this GUI.

## Thanks

To [@gergondet](https://github.com/gergondet) for developing and helping with the mc\_rtc framework.
