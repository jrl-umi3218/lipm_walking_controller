# LIPM Walking Controller

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI of LIPMWalking](https://github.com/jrl-umi3218/lipm_walking_controller/actions/workflows/build.yml/badge.svg)](https://github.com/jrl-umi3218/lipm_walking_controller/actions/workflows/build.yml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/index.html)

[![Stair climbing by the HRP-4 humanoid robot](https://scaron.info/images/stair-climbing.jpg)](https://www.youtube.com/watch?v=vFCFKAunsYM&t=22)

For technical details, please refer to the following work: [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document). If you use the project in your own work, please cite the paper as follows:

```
@inproceedings{caron2019stair,
  title={Stair climbing stabilization of the HRP-4 humanoid robot using whole-body admittance control},
  author={Caron, St{\'e}phane and Kheddar, Abderrahmane and Tempier, Olivier},
  booktitle={2019 International conference on robotics and automation (ICRA)},
  pages={277--283},
  year={2019},
  organization={IEEE}
}
```

This project has been used in the following work, and much more:
- Humanoid robots in aircraft manufacturing: The airbus use cases: [HAL paper](https://hal-lirmm.ccsd.cnrs.fr/lirmm-02303117/document) - [IEEE paper and video - Best paper award](https://ieeexplore.ieee.org/abstract/document/8889461).

```
@article{kheddar2019humanoid,
  title={Humanoid robots in aircraft manufacturing: The airbus use cases},
  author={Kheddar, Abderrahmane and Caron, St{\'e}phane and Gergondet, Pierre and Comport, Andrew and Tanguy, Arnaud and Ott, Christian and Henze, Bernd and Mesesan, George and Englsberger, Johannes and Roa, M{\'a}ximo A and others},
  journal={IEEE Robotics \& Automation Magazine},
  volume={26},
  number={4},
  pages={30--45},
  year={2019},
  publisher={IEEE}
}
```

- [A Cross-Temporal Robotic Dance Performance: Dancing with a Humanoid Robot and Artificial Life](https://hal.science/hal-04755684/) - [video excerpt](https://youtu.be/6QZwARIV_yQ?si=wIzObdp1b4w4sHGF) - [full video](https://youtu.be/iAVdj0rey5M?si=SICcGOgehyC-IwFt)


## Trying the controller

### Docker

You can try a snapshot of the controller directly by running its Docker image.

```sh
xhost +local:docker
docker run -it --rm --user ayumi -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    stephanecaron/lipm_walking_controller \
    lipm_walking --floor
```

This image runs the exact controller we used in 2019 during experiments and industrial demonstrations. Replace `--floor` with `--staircase` for stair climbing.

### Web

You may also try a version of the controller without dynamic simulation using our [online web demo](https://mc-rtc-demo.netlify.app/#robot=JVRC1&controller=LIPMWalking).

## Building with mc-rtc-superbuild

The recommended way to build this project is to use [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild) along with the extension for LIPM walking controller provided in [superbuild-extensions](https://github.com/mc-rtc/superbuild-extensions).

```sh
git clone --recursive https://github.com/mc-rtc/mc-rtc-superbuild.git
cd extensions
git clone --recursive https://github.com/mc-rtc/superbuild-extensions.git
echo "set(EXTENSIONS_DIR ${CMAKE_CURRENT_LIST_DIR}/superbuild-extensions)" > local.cmake
# Install LIPM walking controller and its dependencies
echo "include(${EXTENSIONS_DIR}/controllers/lipm_walking_controller.cmake)" >> local.cmake
# For dynamics simulation with MuJoCo
echo "include(${EXTENSIONS_DIR}/simulation/MuJoCo.cmake)" >> local.cmake
echo "include(${EXTENSIONS_DIR}/gui/mc_rtc-magnum.cmake)" >> local.cmake
cd ..
# Configure the superbuild and install all required system dependencies
cmake --preset relwithdebinfo
# Clone all projects and their dependencies, and build them
# Note by default this will create a workspace folder in the parent directory
# If you wish to change the path or default options, edit CMakePresets.json or create your own preset in CMakeUserPresets.json
cmake --build --preset relwithdebinfo
```

## Usage

### Running the controller

#### MuJoCo simulation (physics simulation)

- To run the controller with MuJoCo simulation use

```sh
mc_mujoco -f ~/lipm_walking_controller/etc/mc_rtc.yaml
```

#### Ticker (no physics simulation)

- To run the controller in ticker mode (no physics simulation) use:

```sh
mc_rtc_ticker -f ~/lipm_walking_controller/etc/mc_rtc.yaml
```

You can modify this file to run with a different supported robot.

- To visualize the controller with ROS (rviz):

```sh
ros2 launch mc_rtc_ticker display.launch
```

or alternatively if you have built `mc_rtc` without ROS support you can use the standalone [mc-rtc-magnum](https://github.com/mc-rtc/mc_rtc-magnum) standalone visualizer:

```sh
mc-rtc-magnum
```

#### Choreonoid simulation (AIST/LIRMM only)

Alternatively, you can use the choreonoid simulator to run the controller.
While you can make it work with any robot, the setup process is tricky and outside the scope of this README.
For users with the appropriate access (AIST and LIRMM members), we provide a docker image pre-configured with choreonoid, `mc_udp` and all robots supported within the organization.

To use it:
1. Add `mc_udp` to the superbuild extensions in `extensions/local.cmake`:

```cmake
AddProject(
  mc_udp
  GITHUB jrl-umi3218/mc_udp
  GIT_TAG origin/master
  DEPENDS mc_rtc
  APT_PACKAGES libmc-udp-dev python-mc-udp python3-mc-udp mc-udp-control
  CMAKE_ARGS -DBUILD_OPENRTM_SERVER=OFF -DBUILD_MC_RTC_CLIENT=ON
)
```
2. Login to github container registry using a personal access token with `read:packages` scope.
   You can find instructions on how to do this [here](https://docs.github.com/en/packages/working-with-a-github-packages-registry/connecting-to-github-packages-with-docker).

   ```sh
   export CR_PAT=your_personal_access_token
   echo $CR_PAT | docker login ghcr.io -u USERNAME --password-stdin
   ```
3. Pull the docker image:
```sh
docker pull ghcr.io/isri-aist/choreonoid-private:jammy-standalone-release-latest
docker tag ghcr.io/isri-aist/choreonoid-private:jammy-standalone-release-latest choreonoid-latest
```
4. Run choreonoid from the docker container:
```sh
xhost +local:docker # allow X-server connections from docker containers
docker run -it -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw choreonoid-latest
docker run -it -e "DISPLAY=${DISPLAY}" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" --network=host choreonoid-latest
# within the container type help for instructions
# for example to run the simulation with the RHPS1 robot, run:
run_choreonoid RHPS1 sim_mc_udp.cnoid
```
5. On your local machine, run the `mc_udp` client to connect to the docker container:

```sh
MCUDPControl -f ~/lipm_walking_controller/etc/mc_rtc.yaml
```

**NOTE:** Only the choreonoid scripts running `mc_udp` server can be used. choreonoid runs isolated within the docker container but shares its network with the host machine. The `mc_udp` client connects to the server running in the docker container, allowing you to control the robot and visualize it in rviz or mc-rtc-magnum.


### Using the controller

To use the controller, go to the `Walking` tab in the GUI. Click on the `Start standing` button (enables stabilization), select a walking plan amongst the default ones, or use the `custom` plan and move its target using the provided marker. Click on the `Start walking` button to start walking. You can also use the `Stop walking` button to stop the robot.

You can refer to the archived [How to use the graphical user interface](https://github.com/stephane-caron/lipm_walking_controller/wiki/How-to-use-the-graphical-user-interface%3F) of @stephane-caron for screenshots and more details on how to use the GUI.

## Documentation

For documentation, you can refer to the following resources:
- The [Doxygen documentation](http://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/index.html) includes some basic tutorials. This is slightly outdated, but still relevant for the most part.
- @stephane-caron's archived [wiki](https://github.com/stephane-caron/lipm_walking_controller/wiki). While archived it remains relevant for the most part.

## Integrate your own robot

Adding your own robot is fairly straightforward. You need to:

1. Have the robot already integrated in `mc_rtc` (see [this tutorial](https://jrl.cnrs.fr/mc_rtc/tutorials/advanced/new-robot.html))
2. Add [robot-specific configuration files](https://jrl.cnrs.fr/mc_rtc/tutorials/introduction/configuration.html#controller-s-robot-specific-configuration) to your robot module You need:
  - `etc/controllers/LIPMWalking/<robot_name>.yaml` with content
  <details>
    <summary>See example file</summary>
  ```yaml
robot_models:
  hrp2_drc:
    swingfoot:
      weight: 2000
      stiffness: 500
    # A default configuration is loaded from the robot module
    # You may overwrite some parameters here. For supported options, see
    # See https://jrl-umi3218.github.io/mc_rtc/json-full.html#MetaTask/LIPMStabilizerTask
    # For example:
    stabilizer:
      dcm_tracking:
        gains:
          prop: 4
          integral: 20
          deriv: 0.5
      admittance:
        cop: [0.02, 0.01]
        # maxVel:
        #   angular: [1., 1., 1.]
        #   linear: [0.5, 0.5, 0.5]
      tasks:
        contact:
          damping: 300.
      dcm_bias:
        biasDriftPerSecondStd: 0.0001
        withDCMBias: true
    sole:
      half_length: 0.108
      half_width: 0.07
      friction: 0.7
plans:
  hrp2_drc:
    ashibumi: # stepping in place
      double_support_duration: 0.2
      single_support_duration: 0.8
      swing_height: 0.04
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
    custom_backward:
      double_support_duration: 0.2
      single_support_duration: 0.8
      step_length: 0.15
      swing_height: 0.05
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
    custom_forward:
      double_support_duration: 0.1
      single_support_duration: 0.7
      step_length: 0.2
      swing_height: 0.04
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
    custom_lateral:
      double_support_duration: 0.2
      single_support_duration: 0.8
      step_length: 0.1
      swing_height: 0.04
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
      mpc:
        weights:
          jerk: 1.0
          vel: [10.0, 300.0]
          zmp: 1000.0
    walk_backward_75cm:
      double_support_duration: 0.2
      single_support_duration: 0.8
      swing_height: 0.05
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [-0.15, -0.105, 0.0]
          ref_vel: [-0.075, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [-0.3, 0.105, 0.0]
          ref_vel: [-0.15, 0.0, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [-0.45, -0.105, 0.0]
          ref_vel: [-0.15, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [-0.6, 0.105, 0.0]
          ref_vel: [-0.075, 0.0, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [-0.75, -0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [-0.75, 0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: LeftFootCenter
    walk_forward_100cm:
      double_support_duration: 0.1
      single_support_duration: 0.7
      swing_height: 0.04
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.2, -0.105, 0.0]
          ref_vel: [0.1, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.4, 0.105, 0.0]
          ref_vel: [0.2, 0.0, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.6, -0.105, 0.0]
          ref_vel: [0.2, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.8, 0.105, 0.0]
          ref_vel: [0.1, 0.0, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [1.0, -0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [1.0, 0.105, 0.0]
          ref_vel: [0.0, 0.0, 0.0]
          surface: LeftFootCenter
    warmup:
      double_support_duration: 0.1
      single_support_duration: 0.7
      swing_height: 0.04
      contacts:
        - pose:
            translation: [0.035, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.035, 0.105, 0.0]
          surface: LeftFootCenter
        - pose:
            translation: [0.035, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.035, 0.105, 0.0]
          surface: LeftFootCenter
    external: # refer to hrp4cr section for a detailed explanation
      double_support_duration: 0.5
      single_support_duration: 0.7
      allowed_planning_time:
        standing: 2.0
        single_support: 0.2
      swing_height: 0.04
      contacts:
        - pose:
            translation: [0.0, -0.105, 0.0]
          surface: RightFootCenter
        - pose:
            translation: [0.0, 0.105, 0.0]
          surface: LeftFootCenter
      leftFootLandingOffset: [0.0, 0.105, 0.0] # x, y, theta
      rightFootLandingOffset: [0.0, -0.105, 0.0] # x, y, theta
  ```
    </details>
  - Install this file, add the following lines to your `CMakeLists.txt`::
  ```cmake
      install(FILES etc/controllers/LIPMWalking/<robot_name>.yaml
            DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/LIPMWalking/)
  ```
    - Additionally, you will need to tune the stabilizer gains for your robot. See [Tuning the stabilizer](https://jrl.cnrs.fr/lipm_walking_controller/doxygen/HEAD/stabilizer.html) for more information. The tuning can be done live from the GUI in the `Stabilizer` tab. You can export a yaml representation of this configuration from here, and add it to your robot module's `_lipm_stabilizer` configuration file, or to the above `robot_models` section in the `etc/controllers/LIPMWalking/<robot_name>.yaml` file. The most important gains are the `dcm_tracking` PID (dcm control) and the foot admittance gains (force control)
  ```yaml
  robot_models:
    hrp2_drc:
      # <YOUR CUSTOM CONFIGURATION>
      # For supported options, see
      # See https://jrl-umi3218.github.io/mc_rtc/json-full.html#MetaTask/LIPMStabilizerTask
      # For example:
      stabilizer:
        dcm_tracking:
          gains:
            prop: 4
            integral: 20
            deriv: 0.5
        admittance:
          cop: [0.02, 0.01]
  ```

## External Footstep Planner

The controller can be used with an external footstep planner through the provided `ExternalFootstepPlannerPlugin`. This can be used to provide additional capabilities such as path planning and collision avoidance, or to control walking using a joystick.

Supported planners are:
- [Hybrid MPC Footstep Planner](https://github.com/antodld/FootSteps_Planner) : recommended, built by default
- [Online Footstep Planner](https://github.com/isri-aist/OnlineFootstepPlanner) : deprecated, ROS1 only. Refer to [](docs/online_footstep_planner.md) for more (deprecated) information on how to use it.

To use it, select the `external` plan in the GUI, and then select the planner you want to use.


## Thanks

Thanks to:

- [@gergondet](https://github.com/gergondet) for developing and helping with mc\_rtc and maintaining this project
- [@arntanguy](https://github.com/arntanguy) for developing and helping with mc\_rtc and maintaining this project
- [@Saeed-Mansouri](https://github.com/Saeed-Mansouri) for bug hunting and discussion around the project - *Best Debugger Award* 🏅
- [@mmurooka](https://github.com/mmurooka) for bug hunting and discussion around the project
- [@mehdi-banallegue](https://github.com/mehdi-benallegue) for his work on maintaining and improving the stabilizer
- [@TsuruMasato](https://github.com/TsuruMasato) for his work regarding the Online Footstep Planner integration
