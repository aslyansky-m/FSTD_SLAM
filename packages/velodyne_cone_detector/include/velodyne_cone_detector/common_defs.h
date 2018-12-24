//
// Created by maxima on 24/12/18.
//

#pragma once

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

using Viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using PointC = pcl::PointXYZRGB;
using CloudC = pcl::PointCloud<PointC>;

namespace LIDAR {

    namespace PointClasses{
        static const int n_classes = 4;
        enum PointClass : u_char {
            inlier = 0,
            ground,
            too_high,
            too_far
        };

        static constexpr u_char colors[n_classes][3] = {
                {0  ,255,0  }, // green
                {0  ,0  ,255}, // blue
                {255,0  ,255}, // magenta
                {255,0  ,0  }  // red
        };
    }

    class ConeDescriptor{
    public:
        Cloud::Ptr cloud;
        Point mean, stddev;
        int count;
        double radius;
        bool valid;
        ConeDescriptor():
            cloud(new Cloud){
        }

        void calculate(){
            count = cloud->size();
            Point sum, sum2;
            sum.x = 0; sum.y = 0; sum.z = 0;
            sum2.x = 0; sum2.y = 0; sum2.z = 0;
            sum.intensity = 0; sum2.intensity = 0;
            for(auto &pt : (*cloud)){
                sum.x += pt.x; sum.y += pt.y; sum.z += pt.z;
                sum.intensity += pt.intensity;
                sum2.x += pt.x*pt.x; sum2.y += pt.y*pt.y; sum2.z += pt.z*pt.z;
                sum2.intensity += pt.intensity*pt.intensity;
            }
            mean.x = sum.x/count;
            mean.y = sum.y/count;
            mean.z = sum.z/count;
            mean.intensity = sum.intensity/count;
            stddev.x = sqrtf(sum2.x/count-mean.x*mean.x);
            stddev.y = sqrtf(sum2.y/count-mean.y*mean.y);
            stddev.z = sqrtf(sum2.z/count-mean.z*mean.z);
            stddev.intensity = sqrtf(sum2.intensity/count-mean.intensity*mean.intensity);
            radius = sqrtf(stddev.x*stddev.x+stddev.y*stddev.y+stddev.z*stddev.z);
            valid = radius < 0.3 && stddev.x < 0.2 && stddev.y < 0.2 && stddev.z < 0.2;
        }
    };

    using Segmentation = std::vector<PointClasses::PointClass>;

};

