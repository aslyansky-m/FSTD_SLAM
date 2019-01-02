# establish connection
# connect to FORMULA1 password: Formula20
ssh nvidia@192.168.2.10 -X -C
# press yes if asked
# password: nvidia
su ubuntu
# password: ubuntu
cd ~/Desktop/

# check sd-card
sudo mount /dev/mmcblk1p1 /media/sdcard/
# check once again
cd /media/sdcard/ros_capture
 
# launch camera
cd ~/catkin_ws && source devel/setup.bash
roslaunch zed_wrapper zed.launch &
# you must see:
# ZED (Init) >> Depth mode: PERFORMANCE
# ZED (Init) >> Video mode: HD720@30

# launch lidar
roslaunch velodyne_pointcloud VLP16_points.launch &

# check that ros topics are created
rostopic list
# make sure you get a lot of zed/ and velodyne/ topics
# check that sensors are working:
rostopic echo /zed/right/image_rect_color                   # expect to see a lot of numbers
# check that lidar is working
rostopic echo /velodyne_points                              # expect to see a lot of numbers


cd /media/sdcard/ros_capture/jan5
rosbag record zed/right/image_raw_color/compressed zed/left/image_raw_color/compressed zed/imu/data zed/pose /velodyne_points

# to validate:
rosbag info <name_of_bag>
