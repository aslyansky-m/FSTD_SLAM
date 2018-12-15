# Simultaneous Localization and Mapping algorithms for Formula Student Driverless @Technion
TODO: add detailed description
TODO: create wiki, move large parts (installation, roadmap) to wiki

# Hardware
- ZED stereo camera
- Velodyne VLP-16 LIDAR
- Nvidia TX-1            - **currently**
- Nvidia Drive PX-2   - future
- TODO: add

# Hardware Setup
TODO: add detailed instructions

# Roadmap

## System
- ~~camera~~
- ~~LIDAR~~
- ~~Nvidia Jetson setup~~
- ~~capture with rosbag~~
- install and run on the car
- Nvidia Drive setup
- install and run on the car
- TODO: update regarding the electric car

## Visual SLAM
- test [SERVO](https://github.com/grafue/SERVO/) on provided [data](https://github.com/grafue/SERVO/tree/master/Examples/ROS)
- add to the main ROS system
- test cone detection
- integrate cone detection
- add to the main system

## LIDAR SLAM
- test the data: [1](https://www.dropbox.com/s/7x75ks6vo2npfv3/AMZ_driverless_2017_dataset.bag.tar.gz?dl=0), [2](https://github.com/eufsa/datasets)
- control LIDAR speed and FOV from ROS
- ground removal
- cone segmentation and clustering
- add nodes to ROS
- cone tracking (?)
- fusion with camera

# Installation
We will work under Ubuntu 16.04 and [ROS] Kinetic

- Jetson or Drive Firmware
TODO: add

- ROS/ [Kinetic] 
Follow the instructions in [Kinetic] 

- ROS/ [velodyne]
``` sh
sudo apt-get install ros-kinetic-velodyne
cd ~/catkin_ws/src/ && git clone https://github.com/ros-drivers/velodyne.git
rosdep install --from-paths src --ignore-src --rosdistro YOURDISTRO -y
cd ~/catkin_ws/ && catkin_make
```
Also, see [VLP16 tutorial]

- ROS/ [zed-ros-wrapper]
Point Cloud Library dependencies:
``` sh
sudo apt-get install libpcl1
sudo apt-get install ros-kinetic-pcl-ros 
```
Rviz IMU plugin: 
``` sh
sudo apt-get install ros-kinetic-rviz-imu-plugin
```
Get the package from Github and put it in your catkin src folder
``` sh
cd ~/catkin/src
git clone https://github.com/stereolabs/zed-ros-wrapper
cd ~/catkin
catkin_make zed-ros-wrapper
source ./devel/setup.bash
```

# Helper
## LIDAR SLAM
Ground removal:
- [lab 30](https://github.com/cse481wi18/cse481wi18/wiki/Lab-30%3A-Introduction-to-point-cloud-processing)
- [lab 31](https://github.com/cse481wi18/cse481wi18/wiki/Lab-31%3A-Planar-segmentation)

Euclidean clustering:
- [lab 32](https://github.com/cse481wi18/cse481wi18/wiki/Lab-32%3A-Euclidean-clustering)
- [pcl](http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php)

 [ROS]: <http://wiki.ros.org/>
 [Kinetic]: <http://wiki.ros.org/kinetic/Installation/Ubuntu>
 [zed-ros-wrapper]: <http://http://wiki.ros.org/zed-ros-wrapper>
 [velodyne]: <http://http://wiki.ros.org/velodyne>
 [VLP16 tutorial]: <http://http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16>
