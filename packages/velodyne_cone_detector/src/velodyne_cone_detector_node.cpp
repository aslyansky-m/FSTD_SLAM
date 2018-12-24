#include "../include/velodyne_cone_detector/velodyne_cone_detector_node.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Publisher marker_pub =
            n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    LIDAR::OutlierFilter filter(marker_pub);
    ros::Subscriber sub = n.subscribe("velodyne_points/", 10, &LIDAR::OutlierFilter::callback, &filter);
    ros::spin();
    return 0;
}