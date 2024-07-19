# Home Service Robot - Robotics Software Engineer

Final Project for Udacity Nanodegree

## Overview

Demonstrate the final project output and its usage.

### Environment Setup

To run this example without issues, the following environment setup is preferred.

|                  |                          |
|------------------|--------------------------|
| Operating System | Ubuntu 16.04             |
| ROS2 Version     | ROS Kinetic              |


#### Folder Structure

```
HOME_SERVICE_ROBOT/catkin_ws/src
├── add_markers                             # Package to simulate obkect to be picked up using markers                         
│   └── src
│       └── add_markers.cpp
│   ├── CMakeLists.txt
│   ├── package.xml
├── my_robot                                # Package for the home made service robot
│   ├── CMakeLists.txt
│   ├── config                          
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   └── local_costmap_params.yaml
│   ├── launch                              # Contains the launch files
│   │   ├── add_markers.launch  
│   │   ├── amcl_demo.launch  
│   │   ├── mapping.launch       
│   │   ├── robot_description.launch 
│   │   ├── view_navigation.launch    
│   │   └── world.launch     
│   ├── maps                                # Contains map file for SLAM     
│   │   ├── map.pgm                         
│   │   └── map.yaml                        
│   ├── meshes                              # Contains mesh file for Lidar
│   │   └── hokuyo.dae                      
│   ├── rviz                                # Contains config files for rviz                        
│   │   └── navigation.rviz
│   ├── urdf                                # Robot and gazebo URDF urdf files
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   ├── worlds                              # Contains the gazebo world
│   |   └── myworld.world
│   ├── CMakeLists.txt
│   └── package.xml
├── my_robot_interfaces                     # Package containing topic messages
│   ├── msg
│   |   └── RobotMoveState.msg
│   ├── CMakeLists.txt
│   └── package.xml
├── pick_objects                            # Package for robot to pick up pbjects
│   ├── src
│   |   └── pick_objects_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── scripts                                 # Robot ROS launch files scripts
│   ├── add_markers.sh
│   ├── home_service.sh             
│   ├── pick_objects.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── LICENSE
├── README.md
```


## Project Setup

1. Clone it to your machine.  `git clone https://github.com/kimsniper/Home_Service_Robot.git`
2. Navigate to Home_Service_Robot/catkin_ws
3. Run `catkin_make`
4. Open a separate terminal
5. Go to `Test Project` section

## Test Project

Navigate to /Home_Service_Robot/catkin_ws/src/scripts make test_slam.sh file executable

### SLAM Testing

1. Make test_slam.sh file executable
    ```bash
    chmod +x test_slam.sh
    ```
2. Execute test_slam.sh
    ```bash
    ./test_slam.sh
    ```
### Navigation Testing

1. Make test_navigation.sh file executable
    ```bash
    chmod +x test_navigation.sh
    ```
2. Execute test_navigation.sh
    ```bash
    ./test_navigation.sh
    ```
### Home Service Robot Testing

1. Make home_service.sh file executable
    ```bash
    chmod +x home_service.sh
    ```
2. Execute home_service.sh
    ```bash
    ./home_service.sh
