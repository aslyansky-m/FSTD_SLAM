# SERVO

**Original Implementation:** [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**Modified:** Ueli Graf, 07.03.2017

**Current version:** 1.0.0 

This is SERVO, a robust SLAM extension to ORB-SLAM2. SERVO relies on additional odometry estimates provided in a ROS environment. At the time of writing, only stereo SLAM mode is supported.

#1. Quick start

##1.1 Install dependencies

[ROS](ros.org)

[catkin_tools](http://catkin-tools.readthedocs.io/en/latest/installing.html)

[Pangolin](https://github.com/stevenlovegrove/Pangolin) including Required Dependencies glew, cmake

```
sudo apt-get install libglew-dev
sudo apt-get install cmake
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j
```

kindr:
```
sudo add-apt-repository ppa:ethz-asl/common
sudo apt-get update
sudo apt-get install ros-indigo-kindr-*
```

[OpenCV](http://opencv.org) **Required at leat 2.4.3. Tested with OpenCV 2.4.11**

[Eigen3](http://eigen.tuxfamily.org) **Required at least 3.1.0**

BLAS and LAPACK:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

##1.2 Install additional odometry software
This software was tested with [rovio](https://github.com/ethz-asl/rovio) which is included as a submodule. If rovio is to be used, this step can be skipped. If a different visual-inertial odometry framework should be used, install it seperately. Reference body velocity (IMU body velocity in case of rovio) should be published to /rovio/odometry and the transformation from reference body to left camera should be published to /rovio/extrinsics0.

##1.3 Clone and build underlying frameworks

Create a catkin workspace, clone the repository and submodules and execute the build script found in the root directory at SERVO/build.sh. Make sure your terminal is setup with the appropriate ROS environment variables.
```
mdir src
catkin init
cd src
git clone --recursive https://github.com/grafue/SERVO.git
cd SERVO
chmod +x build.sh
./build.sh
```
##1.4 Build catkin packages
```
catkin build rovio orb_slam2 -DCMAKE_BUILD_TYPE=Release
```

##1.5 Unpack rosbag

Join the rosbag provided for testing as a split archive and decompress it for better runtime performance.

Open a terminal at SERVO/Examples/ROS/
```
cat SERVO_bag_* > SERVO.bag
rosbag decompress SERVO.bag
```

##1.6 Launch
Use the exemplary launch files found at SERVO/Examples/ROS/launch/

Open a terminal at SERVO/

```
source ../../devel/setup.bash
roslaunch Examples/ROS/launch/orb_slam2.launch
```
launches ORB_SLAM2 using the provided rosbag in original configuration (Stereo/ORB_SLAM2.yaml). Wait for a few seconds until the vocabulary has been loaded, then press space bar in the terminal to start playback.

Use
```
roslaunch Examples/ROS/launch/servo.launch
```
to launch with the proposed changes (Stereo/SERVO.yaml) and verify increased robustness and performance. Wait for a few seconds until the vocabulary has been loaded, then press space bar in the terminal to start playback.

Use the parameter named ORBextractor.nFeatures in the *.yaml files for tuning computational cost.

The current version assumes that rovio is used in parallel (that is, rovio must be running - it is launched automatically when using the provided launch files) and publishes odometry estimates to /rovio/odometry and the transformation from camera frame to odometry reference frame to /rovio/extrinsics0.
rosbag can be selected within the launch file. If the resulting estimated trajectory is implausible, try decrease the rate (-r parameter for the rosbag node section in the launch files) or tune the compational complexity by changing the ORBextractor.nFeatures parameter in the *.yaml settings files found at Examples/ROS/Stereo/
Topics can be changed by changing the remap command in launch files or by changing the source code at ros_stereo.cc.

#2 Options

Most configuration parameters can bechanged by a .yaml-file found in Examples/Stereo. Use the launch file to specitfy the .yaml file that should be loaded. The parameter file contains short descriptions of the influence of each variable.

#3 Available Topics
Additional to all topics of the respective base implementation:

/orb_slam2/odometry: odometry with respect to the left camera (cam0)

#4 Changes/Implementation

The extension implements a supported tracking procedure for ORB_SLAM2 that incorporates visual-inertial information in the SLAM feature matching and map building process. To this end, VI-odometry is used to support the matching process by providing a pose initial guess at each timestep. Furthermore, the external pose estimate serves as a pose prior in the pose optimization and helps performs a dead reckoning method for contiunous tracking even when no features are matched successfully.

Implementation details will be published soon.

#5 Original Documentation
For extensive and additional documentation on the base implementations of the frameworks used, see the original repositories of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) and [rovio](https://github.com/ethz-asl/rovio).
