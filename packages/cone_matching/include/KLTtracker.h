//
// Created by maxima on 26/01/19.
//

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/optflow.hpp"
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <algorithm>



class KLTtracker{

public:
    class Params {
    public:
        int winSize = 10;
        int maxLevel = 5;
        int flags = 0;
        int flags2 = 0;
        int criteriaMaxIter = 30;
        double criteriaEps = 0.001;
        double minEigThreshold = 1e-4;
        bool retrack = true;
        double retrack_err = 5.0;
        Params(){}
        Params(int winSize,int maxLevel) : winSize(winSize), maxLevel(maxLevel){}
    };
public:
    Params params;
protected:
    bool is_first = true;
    std::vector<cv::Mat> prevPyr, nextPyr;
    cv::Ptr<cv::SparsePyrLKOpticalFlow> optflow;
public:
    KLTtracker(Params params_ = {}) : params(params_) {
        optflow = cv::SparsePyrLKOpticalFlow::create(cv::Size(params.winSize, params.winSize), params.maxLevel,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, params.criteriaMaxIter, params.criteriaEps),
                params.flags,params.minEigThreshold);
    }

    inline bool operator()(const cv::Mat &image, const std::vector<cv::Point2f> &prev_points, std::vector<cv::Point2f> &next_points, std::vector<uchar> &success){
        return track(image,prev_points,next_points,success);
    }


protected:
    bool track(const cv::Mat &image, const std::vector<cv::Point2f> &prev_points, std::vector<cv::Point2f> &next_points, std::vector<uchar> &success) {
        if (is_first) {
            cv::buildOpticalFlowPyramid(image, prevPyr, cv::Size(params.winSize, params.winSize), params.maxLevel, true);
            is_first = false;
            return false;
        }

        if (prev_points.size() == 0)
            return false;

        cv::buildOpticalFlowPyramid(image, nextPyr, cv::Size(params.winSize, params.winSize), params.maxLevel, true);
        cv::Mat status1, status2, err1, err2;

        optflow->calc(prevPyr,nextPyr,prev_points,next_points,status1,err1);

        auto N = prev_points.size();
        success.resize(N);
        mat_to_vec(status1,success);

        if (params.retrack) {
            std::vector<cv::Point2f> prev_points_aux;
            if (params.flags2!=0) {
                prev_points_aux = prev_points;
            }
            optflow->calc(nextPyr, prevPyr,next_points, prev_points_aux,status2, err2);

            for (int i = 0; i < prev_points.size(); i++) {
                cv::Point2f diff = (prev_points_aux[i] - prev_points[i]);
                float err = abs(diff.x) + abs(diff.y);
                if (!*status2.ptr<uchar>(i) || err > params.retrack_err) {
                    success[i] = 0;
                }
            }
        }

        std::swap(nextPyr, prevPyr);
        return true;
    }


    static void mat_to_vec(cv::Mat &mat, std::vector<uchar>& array) {
        if (mat.isContinuous()) {
            array.assign(mat.datastart, mat.dataend);
        }
        else {
            for (int i = 0; i < mat.rows; ++i) {
                array.insert(array.end(), mat.ptr<uchar>(i), mat.ptr<uchar>(i) + mat.cols);
            }
        }
    }

};