# Re-organized LIPM Walking Controller for Joystick Walking

[![demo video](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2019-17-45.png)](https://youtu.be/XoXDZBgbamc)


Video link is [here](https://youtu.be/XoXDZBgbamc)

This version of LIPM walking controller is mainly developed by [Arnaud Tanguy](https://github.com/arntanguy).

He fixed the "double stepping" problem, communication delay between several PCs for the experiment, and so on.


And then, [Masato Tsuru](https://github.com/TsuruMasato) added an interface for PS4 controller, Oculus controller, and RViz gui menu.

If you want to try walking HRP4CR with PS4 Joystick, this version must be the easiest way.


___
## Required environments

Required components and libraries :
* Ubuntu 18.04 LTS
* ROS melodic (desktop full)
* mc_rtc, the latest version (2021/1/18) https://github.com/jrl-umi3218/mc_rtc
* openrtp environment (private repository)
* HRP4CR robot model and its mc_rtc support (private repository)
* Online Footstep Planner, the latest version (2021/1/18) https://github.com/isri-aist/OnlineFootstepPlanner
* PS4 controller and USB cable

__Basically, drcutl's environment setup script and mc_rtc installation script automatically set up most of those libraries.__

__Only "OnlineFootstepPlanner" is not automatically installed by those scripts.__

About the installation of OnlineFootstepPlanner, please visit [its page](https://github.com/isri-aist/OnlineFootstepPlanner)

___
## How to build

#### 1. download this repository:

``$ git clone git@github.com:TsuruMasato/lipm_walking_controller.git `` (default branch "rebase_stabilizer_ana")


#### 2. go into the repository and make "build" directory:

``$ mkdir build``


#### 3. go into the build directory and run ccmake :

``$ ccmake ..``

Turn on **AUTOLOAD_ExternalFootstepPlannerPlugin** option.

And also, don't forget to set **CMake Install prefix** to /home/*your_name*/openrtp.


#### 4. build this controller, and install it :

``$ make -j8``

``$ make install``


___
## How to run


First, please switch the mc_controller in your mc\_rtc configuration as doing below :

``$ nano ~/.config/mc_rtc/mc_rtc.yaml``

```yaml
MainRobot: HRP4CR
Timestep: 0.002
Enabled: LIPMWalking
```


### You need at least these 4 terminal windows.

* ROS core
* Choreonoid simulation
* RViz as mc_rtc control interface
* Online Footstep Planner


***

### process


#### 1. start ROS core in 1th terminal :

``$ roscore ``


#### 2. Please go to the HRP4CR directory in openrtp system in 2nd terminal :

``$ cd ~/openrtp/share/hrpsys/samples/HRP4CR``


#### 3. Start Choreonoid simulation in 2nd terminal :

``$ choreonoid sim_mc_openrtp_bush.cnoid ``

and please click the starting button in Choreonoid.

Now, the LIPM walking controller is running.
The HRP4CR robot model will keep standing.



_(if the robot fails on the ground, `$ clear-omninames.sh` sometimes solve it.)_

_(Even clear-omninames.sh cannot help you, your hrpsys-humanoid is too old. please update all libraries with drcutl script.)_


![standing HRP4CR](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-28-38.png)



#### 4. Start RViz in 3rd terminal :

`` $ roslaunch mc_rtc_ticker display.launch ``

![RViz panel](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-28-46.png)


#### 5. Plug your PS4 controller to a USB port, and start Online Footstep Planner in 4th terminal :

`` $ roslaunch online_footstep_planner run_demo_in_sim_rebase.launch ``

Joystick connection node and Online Footstep Planner node start.

If you can see the yellow warning message "received a map" like below-right side of the picture, it's success!!


![terimal_message](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-29-11.png)


___

___

## GUI menu

After starting the whole system, you have to select some GUI options.

#### 1. Go to "Walking" tab in the control panel in RViz


![Walking tab in RViz](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-40-29.png)


#### 2. Click "start standing" button.

_The stabilizer task becomes enable, and HRP4CR starts balancing._


#### 3. Set Footstep plan menu to "external"

#### 4. Change Target type to "PS4 Controller"

![Final state of RViz panel](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-41-16.png)


#### 5. Now you can walk the robot via Joysticks!!


##### Left Joystick leads the robot walking foward/backward/left/right, and Right Joystick makes him turn.
