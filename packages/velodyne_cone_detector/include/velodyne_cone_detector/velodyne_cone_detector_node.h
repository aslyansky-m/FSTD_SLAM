//
// Created by maxima on 24/12/18.
//

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

#include <pcl/common/angles.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/filters/extract_indices.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/PointIndices.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <shape_msgs/SolidPrimitive.h>
#include "visualization_msgs/Marker.h"


#include "common_defs.h"
#include "cone_visualizer.h"


#include <cmath>


namespace LIDAR {

    class OutlierFilter{
    public:
        class Params{
        public:
            bool z_threshold_enable = true;
            float z_threshold_min = -0.5f;
            float z_threshold_max = 1.0f;
            Params(){}
        };

    protected:
        Params params;
        Visualizer vis;
        ros::Publisher marker_pub;
    public:
        OutlierFilter(ros::Publisher marker_pub, Params params = Params()) :
                params(params),marker_pub(marker_pub){
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

            //ROS_INFO("size: %d, point_step: %d, row_step: %d, width: %d, height: %d]", (int)msg->data.size(), (int)msg->point_step, (int)msg->row_step, (int)msg->width, (int)msg->height);
            Cloud::Ptr cloud_in(new Cloud), cloud_out(new Cloud);

            pcl::fromROSMsg(*msg, *cloud_in);

            // filter
            otlier_filter_impl(cloud_in, cloud_out);

            // display


        }

    protected:
        void otlier_filter_impl(Cloud::Ptr &cloud_in, Cloud::Ptr &cloud_out) {

            { // remove distant object
                pcl::PassThrough<Point> pass_z;
                pass_z.setInputCloud(cloud_in);
                pass_z.setFilterFieldName("z");
                pass_z.setFilterLimits(-1.0f, 2.0f);
                pass_z.filter(*cloud_out);
            }

            pcl::ModelCoefficients::Ptr plane_coefs (new pcl::ModelCoefficients);
            { // RANSAC plane estimation
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

                // Create the segmentation object
                pcl::SACSegmentation<Point> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.01);
                // at most 15 degrees from z axis
                seg.setAxis(Eigen::Vector3f(0, 0, 1));
                seg.setEpsAngle(pcl::deg2rad(15.0));

                seg.setInputCloud(cloud_out);
                seg.segment(*inliers, *plane_coefs);
            }

            Segmentation segm(cloud_out->size());
            Cloud::Ptr cloud_cones(new Cloud);
            cloud_cones->reserve(cloud_out->size());
            { // segment the cones
                for (int n = 0; n < cloud_out->size(); n++) {
                    Point &pt = (*cloud_out)[n];
                    float height = (float) pcl::pointToPlaneDistanceSigned(pt,plane_coefs->values[0],
                                                                           plane_coefs->values[1],
                                                                           plane_coefs->values[2],
                                                                           plane_coefs->values[3]);
                    float distance = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

                    auto &s = segm[n];
                    if ((height < 0.01) || (distance < 7.0 & height < 0.03))
                        s = PointClasses::ground;
                    else if (distance > 40.0) {
                        s = PointClasses::too_far;
                    } else if (height > 0.5) {
                        s = PointClasses::too_high;
                    } else {
                        s = PointClasses::inlier;
                        cloud_cones->push_back(pt);
                    }
                }
            }

            std::vector<ConeDescriptor> cones;
            SegmentConeInstances(cloud_cones, cones);

            vis.draw(cloud_in, cloud_out, segm, plane_coefs, cones);
        }


        void SegmentConeInstances(Cloud::Ptr cloud_cones, std::vector<ConeDescriptor> &cones) {

            double cluster_tolerance;
            int min_cluster_size, max_cluster_size;
            ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.3);
            ros::param::param("ec_min_cluster_size", min_cluster_size, 1);
            ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

            std::vector<pcl::PointIndices> object_indices;
            pcl::EuclideanClusterExtraction<Point> euclid;
            euclid.setInputCloud(cloud_cones);
            euclid.setClusterTolerance(cluster_tolerance);
            euclid.setMinClusterSize(min_cluster_size);
            euclid.setMaxClusterSize(max_cluster_size);
            euclid.extract(object_indices);

            pcl::ExtractIndices<Point> extract;
            extract.setInputCloud(cloud_cones);

            cones.reserve(object_indices.size());
            size_t min_size = std::numeric_limits<size_t>::max();
            size_t max_size = std::numeric_limits<size_t>::min();
            pcl::PointIndices::Ptr object_ptr(new pcl::PointIndices);
            for (auto& object: object_indices) {
                size_t cluster_size = object.indices.size();
                if (cluster_size < min_size) {
                    min_size = cluster_size;
                }
                if (cluster_size > max_size) {
                    max_size = cluster_size;
                }
                ConeDescriptor cone;
                *object_ptr = object;
                extract.setIndices(object_ptr);
                extract.filter(*cone.cloud);
                cone.calculate();
                cones.push_back(cone);
            }

            ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
                     object_indices.size(), min_size, max_size);
        }
    };
};
