# sdi_turtlebot3_toy_app_ws
- [sdi\_turtlebot3\_toy\_app\_ws](#sdi_turtlebot3_toy_app_ws)
  - [1. Introduction {#introduction}](#1-introduction-introduction)
  - [2. Design {#design}](#2-design-design)
    - [Autoware](#autoware)
    - [Dummy ROS nodes](#dummy-ros-nodes)
    - [Simple user interface nodes](#simple-user-interface-nodes)
  - [3. Prerequisite {#prerequisite}](#3-prerequisite-prerequisite)
  - [4. Build {#build}](#4-build-build)
  - [5. Run {#run}](#5-run-run)
    - [Quick start](#quick-start)
    - [Debug](#debug)
  - [6. Result {#result}](#6-result-result)
  - [7. Final remarks {#final}](#7-final-remarks-final)
    - [Contact to authors](#contact-to-authors)



## 1. Introduction {#introduction}
This is a workspace of ROS2 turtlebot3 toy application for SDI project.

This toy application is designed
* to provide a reusable source skeleton of a ROS2 python application for robot mobility (e.g., turtlebot3), 
* to show blueprints of robot mobility software based on the [Autoware project](https://autoware.org/),
* to facilliate study about the concept of ROS2 and Autoware, and
* last but not least, to provide a working dummy mobility application for collaboration between research partners of SDI project.



## 2. Design {#design}
![Toy application design (in ROS graph form)](docs/figures/toy_app_design.jpg)

In this page, we breifly introduce three key concepts of the design of this toy application.

### Autoware
Our toy application mimics [the autonomous driving system architecture of Autoware](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/).
Autoware's autonomous driving system architecture consists of 7 core modules.
To utilize turtlebot3, we reuse the ROS2 nodes provided by turtlebot3 for the sensing, vehicle interface, and map modules.

### Dummy ROS nodes

We provide our own toy ROS2 nodes for the localization, perception, planning, and control modules.
The nodes included in the toy application are working dummy nodes that do not provide full autonomous driving functionality.
Currently, as an example, they only reference image and lidar data, and most of the outputs from each node are siimple strings.
However, the flow of the nodes reflects Autoware's design and is useful for understanding the basic flow of ROS2 and Autoware in a simple manner.
This can be expanded to develop a real application based on Autoware. For more information, refer to the [Autoware design documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/).


### Simple user interface nodes
An autonomous driving system can provide a user interface to the user. Therefore, we provide user interface ROS nodes (i.e., service and action clients) that express several predictable scenarios. 1) `/my_object_detection_service`: returns a list of objects recognized by the robot to the user, 2) `/my_destination_action`: user requests a destination they want to go to, 3) `/my_emergency_service`: switches to manual driving in case of an emergency. These nodes are also working dummy nodes.


## 3. Prerequisite {#prerequisite}
While this repository is a standalone ROS2 workspace that can be build and run without Turtlebot3, it is recommended to run it with Turtlebot3 simulation or real device.

This toy application is dependent on followings:

* ROS2 (https://docs.ros.org/)
* Gazebo (https://classic.gazebosim.org/tutorials?cat=connect_ros)
* Gazebo-ROS bridge package (https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
* Turtlebot3 (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

Make sure you install and build above denepndencies.

This instruction does not covers manuals of above dependencies.
If you satisfied all prerequesites, you may success to follow all steps of following Turtlebot3-ROS2-Gazebo tutorials.

* on real world: https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation
* on simulation: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation

You may see your Turtlebot3 works on your real or virtual world.

## 4. Build {#build}

Follow the steps to build our toy application.

1. Clone our toy application workspace.

```bash
{your_directory}$   git clone https://github.com/yongjunshin/sdi_turtlebot3_toy_app_ws.git
```

2. Build all packages in the workspacae

```bash
{your_directory}$   cd sdi_turtlebot3_toy_app_ws
{your_directory}/sdi_turtlebot3_toy_app_ws$     colcon build --symlink-install
```

This will show following outputs.
```console
xxx@xxx:~/sdi_turtlebot3_toy_app_ws$ colcon build --symlink-install
Starting >>> ros_node_interface
Starting >>> bringing_up
Finished <<< ros_node_interface [0.22s]                                     
Starting >>> control
Starting >>> localization
Starting >>> perception
Starting >>> planning
Finished <<< bringing_up [0.81s]                                           
Finished <<< control [0.76s]                                                
Finished <<< perception [0.76s]
Finished <<< planning [0.76s]
Finished <<< localization [0.77s]

Summary: 6 packages finished [1.11s]

```

## 5. Run {#run}

Before run our toy application, run turtlebot3 application first in virtual or real world, as you did in the [prerequisite section](#prerequisite)

There are many ways to run applications in ROS2 (e.g., launch, run, etc.).
Here we introduce two simple ways to launch our toy application: "quick start" and "debug".
Both result in the same state.


### Quick start

To quickly bring up all toy application nodes all at once, launch bringing up.

Note: '*Bringing up*' package is a typical approach in ROS project to include some launch files of execution scenarios that use many ROS nodes spreaded in multiple packages.

1. Launch `my_toy_bringing_up.launch.py` in the workspace.

```bash
{your_directory}/sdi_turtlebot3_toy_app_ws$     ros2 launch bringing_up my_toy_bringing_up.launch.py
```

### Debug

To carefully see the logs of different moduels in seperate shell, execute launch files for each pakcage on different shell.

1. Launch `my_localization.launch.py` in the workspace.

```bash
{your_directory}/sdi_turtlebot3_toy_app_ws$     ros2 launch localization my_localization.launch.py
```

2. Launch `my_perception.launch.py` in the workspace.

```bash
{your_directory}/sdi_turtlebot3_toy_app_ws$     ros2 launch perception my_perception.launch.py
```

3. Launch `my_planning.launch.py` in the workspace.

```bash
{your_directory}/sdi_turtlebot3_toy_app_ws$     ros2 launch planning my_planning.launch.py
```

4. Launch `my_control.launch.py` in the workspace.

```bash
{your_directory}/sdi_turtlebot3_toy_app_ws$     ros2 launch control my_control.launch.py
```

If you want to go more deep, you can run (`ros2 run ...`) all ROS nodes defined in our workspace one by one manually without using launch files.


## 6. Result {#result}

Checking `rqt-graph` is one of the most simplest to see the status of our toy application.

```bash
{your_directory}/sdi_turtlebot3_toy_app_ws$     rqt-graph
```

![ROS graph of running toy application](docs/figures/toy_app_ros_graph.png)

You can see the result of `rqt-graph` is mapped to the [design of toy application](#design).


## 7. Final remarks {#final}

Now you have a running dummy toy application that covers ROS2, Autoware, and Turtlebot3.

You can extend this skeleton application for your purpose.

Enjoy!

### Contact to authors

* Dr. Yong-Jun Shin: yjshin@etri.re.kr