## AVP_MAPPING

A mapping system for autonomous valet parking(AVP).

Developing [-----> 20 % -----------------------]

- [ ] Simulation
  - [x] static world
  - [ ] dynamic agents 
- [ ] Data pretreatment
  - [x] vidar point cloud
  - [x] bev image
  - [ ] occupied grid for submap
- [ ] Front End
- [ ] Back End
- [ ] Loop Closing
- [ ] Mapping & Visualization 


### Quick Start

**Clone and build**
 
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:adin-pro/avp_mapping.git
cd ..
catkin_make
source devel/setup.zsh
```

**Prepare model & materials for simulation**

unzip parklot.tar.gz and copy extraced files to .gazebo/models/

```shell
tar -zxvf parklot.tar.gz;
cp -r parklot/ .gazebo/models/
```

**Launch Simulation world and rviz**

replace `word_dir`  in `avp_mapping/config/global_config.yaml` with your own path

```shell
roslaunch avp_mapping online_simulation.launch
```

**Control your robot**

```shell
roslaunch avp_mapping robot_control.launch
rosrun avp_mapping avp_data_pretreat_node
```

![alt text](pics/online_simulation.png "rviz")


**Mapping With GroundTruth Odom**

replace `save_path`  in `avp_mapping/config/mapping/auxiliary.yaml` with your own path

```shell
rosrun avp_mapping avp_auxiliary_node
```




Acknowlegement:

1. Code FrameWork https://github.com/Little-Potato-1990/localization_in_auto_driving
2. Gazebo simulation environment https://github.com/TurtleZhong/AVP-SLAM-SIM
3. Robot Control Module https://github.com/huchunxu/ros_exploring
4. Implementation Reference https://github.com/liuguitao/AVP-SLAM-PLUS
