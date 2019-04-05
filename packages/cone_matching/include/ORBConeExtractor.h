/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include "Common.h"


namespace ORB_SLAM2
{



class ORBextractor
{
public:


    ORBextractor(int nfeatures, float margin, bool calcAngles);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray _image, const std::vector<bbox_t>& _objects,
                     std::vector<cv::KeyPoint>& _keypoints,cv::OutputArray _descriptors, std::vector<cv::Rect> rects, std::vector<cv::Mat> images);


protected:

    std::vector<cv::Point> pattern;

    int nfeatures;

    float margin;

    bool calcAngles = false;

    int nclasses = 3;

    std::vector<int> umax;

};

} //namespace ORB_SLAM

#endif

