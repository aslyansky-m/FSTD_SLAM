//
// Created by maxima on 24/12/18.
//

#pragma once

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "common_defs.h"

class Visualizer{
public:
    Viewer viewer;
    int v1 = 0;
    int v2 = 0;

    int point_size = 5;

    const char* const c1 = "original";
    const char* const c2 = "processed";

    Visualizer(){
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3d visualizer"));
        viewer->initCameraParameters ();
        viewer->setCameraPosition (/*pos_xyz*/0, 20, 20, /*view_xyz*/0, 0, -2, /*up_xyz*/0, -1, -1);

        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText(c1, 10, 10, "v1 text", v1);

        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.0, 0.0, 0.0, v2);
        viewer->addText(c2, 10, 10, "v2 text", v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, c1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, c2);
        viewer->addCoordinateSystem (1.0);
    }

    void draw(Cloud::ConstPtr cloud, Cloud::ConstPtr cloud_out, LIDAR::Segmentation &segm, pcl::ModelCoefficients::Ptr plane_coefs, std::vector<LIDAR::ConeDescriptor> &cones){

        CloudC::Ptr cloud_color(new CloudC);
        cloud_color->resize(cloud_out->size());
        for(int n = 0; n < cloud_out->size(); n++){
            auto &c = (*cloud_color)[n];
            auto &p = (*cloud_out)[n];
            c.x = p.x; c.y = p.y; c.z = p.z;
            const u_char * rgb = LIDAR::PointClasses::colors[segm[n]];
            c.r = rgb[0]; c.g = rgb[1]; c.b = rgb[2];
        }

        pcl::visualization::PointCloudColorHandlerGenericField<Point> intensity(cloud, "intensity");
        pcl::visualization::PointCloudColorHandlerRGBField<PointC> segmanetation(cloud_color);

        if( !viewer->updatePointCloud<Point>( cloud, intensity, c1) ){
            viewer->addPointCloud<Point> (cloud, intensity, c1, v1 );
        }
        if( !viewer->updatePointCloud<PointC>( cloud_color,segmanetation, c2) ){
            viewer->addPointCloud<PointC>( cloud_color,segmanetation, c2, v2 );
        }

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, c1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, c2);

        viewer->removeAllShapes();
        viewer->addPlane(*plane_coefs, "plane",v2);

        int n = 0;
        for(auto& cone: cones){
            if(cone.valid){
                std::string name = "sphere_" + std::to_string(n++);
                viewer->addSphere(cone.mean, 0.3, 1.0, 0.0, 0.0, name, v2);
            }
        }

        viewer->spinOnce(1);
    }
};

