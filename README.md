# Simultaneous Localization and Mapping algorithms for Formula Student Driverless @Technion
TODO: 
- add detailed description
- add a nice pic

Also, see our [wiki](https://github.com/aslyansky-m/FSTD_SLAM/wiki)

## Table of Contents
- [Installation](https://github.com/aslyansky-m/FSTD_SLAM/wiki/Installation)  
- [Roadmap](#Roadmap) 
  - [Visual Cone Detection](#Visual-Cone-Detection)
  - [Visual SLAM Exploration](#Visual-SLAM-Exploration)
  - [Visual SLAM System](#Visual-SLAM-System)
  - [LIDAR Setup](#LIDAR-Setup)
  - [LIDAR Cone Detection](#LIDAR-Cone-Detection)
  - [LIDAR SLAM Exploration](#LIDAR-SLAM-Exploration)
  - [LIDAR Landmark Based SLAM](#LIDAR-Landmark-Based-SLAM)
  - [Model Predictive Control](#Model-Predictive-Control)
  - [System](#System)
  - [Simulator](#Simulator)
  - [Hardware ](#Hardware)


# Installation
See [wiki/Installation](https://github.com/aslyansky-m/FSTD_SLAM/wiki/Installation)

# Roadmap
Currently working on **bold**

## Visual Cone Detection
- [ ] **annotate real data**
  - see [instructions](https://github.com/aslyansky-m/FSTD_SLAM/blob/master/doc/cone_dataset.md)
- [ ] **train [YOLOv3-tiny](https://github.com/qqwweee/keras-yolo3)**
- [ ] test inference
- [ ] setup [AirSim](https://github.com/FSTDriverless/AirSim)
- [ ] add cone position information
- [ ] dataset generation pipeline
- [ ] augmentation
  - [ ] noise 
  - [ ] low light
  - [ ] weather
- [ ] generate dataset
- [ ] retrain the network
- [ ] test in real-world
- [ ] test inference performance on Xavier
- [ ] improvements:
  - [ ] add sub-pixel refinement
  - [ ] add tracking

## Visual SLAM Exploration
- [x] test [orbslam 2](https://github.com/raulmur/ORB_SLAM2.git)
  - discarded as not robust enough
- [x] test [orbslam_dwo](https://github.com/JzHuai0108/ORB_SLAM)
  - discarded since no RT branch available, also slower than original orbslam
- [x] test [rovio](https://github.com/ethz-asl/rovio)
  - works OK on servo data
- [x] test [servo](https://github.com/grafue/SERVO/) on provided [data](https://github.com/grafue/SERVO/tree/master/Examples/ROS)
  - [x] bugfixes - doesn't compile
  - [x] compile
  - [x] test ROS node on provided data - works fine
- [x] conclusion: **continue working with servo**

## Visual SLAM System
- [ ] integrate servo with the main system
- [ ] calibrate our cameras and IMU
- [ ] test on our data
- [ ] performance optimization
  - [ ] integrate with [orbslam2-gpu](https://github.com/yunchih/ORB-SLAM2-GPU2016-final)
  - [ ] add [binary vocabulary](https://github.com/raulmur/ORB_SLAM2/pull/21)
  - [ ] remove Pangolin GUI
- [ ] publish points and poses to ROS
- [ ] test cone detection
- [ ] implement cone detector and descriptor
- [ ] implement data fusion in orbslam
- [ ] test on the car

## LIDAR Setup
- [x] install ROS velodyne drivers
- [x] test and store data
- [ ] control LIDAR speed from ROS, see [here](http://wiki.ros.org/velodyne_driver)
- [ ] filter LIDAR by FOV and distance, search for ring information, see [here](http://wiki.ros.org/velodyne_pointcloud)
- [ ] **LIDAR-camera calibration**
  - [x]  [lidar_camera_calibration](http://wiki.ros.org/lidar_camera_calibration) doesn't compile, todo: debug
  - [ ] try [this](https://github.com/agarwa65/lidar_camera_calibration)
- [ ] fusion with camera

## LIDAR Cone Detection
- [x] test the [data](https://github.com/aslyansky-m/FSTD_SLAM/wiki/Datasets)
- [x] add node to ROS 
- [X] ground removal
- [X] cone segmentation and clustering
- [ ] cone tracking
  - probably redundant
- [ ] additional filtering
- [ ] color from intensity
- [ ] color from camera
- [ ] improvements
  - [SqueezeSeg](https://github.com/BichenWuUCB/SqueezeSeg)

## LIDAR SLAM Exploration
- [x] test off the shelf LIDAR SLAM solutions
  - [x] [loam_velodyne](https://github.com/laboshinl/loam_velodyne) - not good enough for tracking cones
  - [x] [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) - doesn't compile
  - [x] [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - worse than loam_velodyne
  - [x] [cartographer](https://github.com/googlecartographer/cartographer) - compiled, looks promising, but integration is difficult
- [ ] next will unlikely work
  - [ ] [mrpt](https://www.mrpt.org/list-of-mrpt-apps/application-kf-slam/)
  - [ ] [gmapping](http://wiki.ros.org/gmapping)
  - [ ] [hector_mapping](http://wiki.ros.org/hector_mapping) - similar to gmapping
- [x] conclusion: general-purpose LIDAR SLAM is not good enough

## **LIDAR Landmark Based SLAM**
 - [ ] explore directions
   - [ ] Kalman Filter vs Graph Optimization
   - [ ] [g2o](https://github.com/RainerKuemmerle/g2o) vs [gtsam](https://bitbucket.org/gtborg/gtsam)

## System
- [ ] define ROS node topology
- [ ] add documentation 

## Simulator 
- [ ] [eufs_sim](https://github.com/eufsa/eufs_sim) - gazebo simulator
  - [x] compiled
  - [ ] bugfix - no car
- [ ] [AirSim](https://github.com/FSTDriverless/AirSim) - Microsoft's framework
  - [ ] test on linux
  - [ ] add lidar, see [here](https://github.com/Microsoft/AirSim/blob/master/docs/lidar.md)
  - [ ] add IMU, see [here](https://github.com/Microsoft/AirSim/tree/master/AirLib/include/sensors/imu)
  - [ ] integrate with ROS, see [here](https://github.com/Microsoft/AirSim/blob/master/docs/ros.md)
  
## Hardware
- [x] camera - ZED
- [x] LIDAR - Velodyne VLP-16
- [x] Nvidia Jetson setup
- [x] capture with rosbag
- [x] install and run on the car
- [x] capture demo content 
- [x] capture real content
- [X] ~~Nvidia Drive PX2 setup~~ 
- [X] after meeting with Nvidia decided to use Jetson AGX Xavier
- [ ] improve capture
  - [ ] find better camera + IMU, see [camera_considerations](doc/camera_considerations.md)
  - [ ] debug low fps
- [ ] automate capture - launch files
- [ ] capture new content
- [ ] Nvidia Jetson Xavier setup
- [ ] install and run on the car
- [ ] TODO: update regarding the electric car
  - [ ] sensors
  - [ ] control
  
## Model Predictive Control
- [ ] explore different options:
  - [MPCC](https://github.com/alexliniger/MPCC) Model Predictive Contouring Controller for Autonomous Racing
  - [MPPI](https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-(MPPI)) Model Predictive Path Integral Controller
- [ ] start working on simulator


