# Final Project: Intro To Robotics 
- Repository: UCM_CSE_180_FINAL
- Authors: Mike Monokandilos, Milan Overholtzer, Jerin Sajimon
- Instructor: StefanoCarpin

# RELEVANT FINAL PROJECT COMMANDS
### Running Final Project
-  OPEN 1ST TERMINAL TO RUN GAZEBO
    1. export TURTLEBOT3_MODEL=waffle
    2. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~Desktop/MRTP/MRTP/src/gazeboenvs/models
    3. ros2 launch gazeboenvs tb3_simulation.launch.py


-  OPEN SECOND TERMINAL TO RUN PROJECT
    1. colcon build
    2. . install/setup.bash
    3. ros2 run project navigate


    
#### LAB07
- not sure if this is what we are supposed to use
    1.  export TURTLEBOT3_MODEL=waffle
    2. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
    3. ros2 launch gazeboenvs tb3_simulation.launch.py 


# WORKSPACE SETUP: FOXY, GAZEBO, UBUNTU, AND C++ CODE
### Creating a Workspace
- 
    1. mkdir -p CSE180/src
    2. colcon build
    3. . install/local_setup.bash


### Creating a Package
-  
    1. ros2 pkg create [name_of_pkg]
    2. ros2 pkg create --build-type ament_cmake [name_of_pkg]


### Download ubuntu at this link:
- https://www.releases.ubuntu.com/focal/

### Download foxy at this link:
- https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html


#   Instructions taken from STEFANO CARPIN: https://github.com/stefanocarpin/MRTP
-   1. Install ROS2 Foxy: follow instructions for Desktop Install on https://docs.ros.org/en/foxy/  Installation/Ubuntu-Install-Debians.html.
    2. Install Gazebo: sudo apt install ros-foxy-gazebo-*
    3. Install the turtlebot packages: sudo apt install ros-foxy-turtlebot3* (you can skip this step if you do the optional step described below)
    4. Install the nav2 packages: sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup (see also https://navigation.ros.org/build_instructions/index.html for more details)
    5. Install colcon: follow instructions on https://colcon.readthedocs.io/en/released/user/installation.html
    6. Optional: Install additional ROS2 packages: sudo apt install ros-foxy-* (be patient -- this will take some time and a lot of space!).
    7. It is advised that you add the following command to your startup scripts (e.g., .basrc) source /opt/ros/foxy/setup.bash

