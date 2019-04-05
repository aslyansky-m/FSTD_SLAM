#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include "opencv2/core/version.hpp"
#include "opencv2/videoio/videoio.hpp"

#include "KLTtracker.h"
#include "Detector.h"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using namespace std;

vector<string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE"};

// create tracker by name
cv::Ptr<cv::Tracker> createTrackerByName(string trackerType)
{
    cv::Ptr<cv::Tracker> tracker;
    if (trackerType ==  trackerTypes[0])
        tracker = cv::TrackerBoosting::create();
    else if (trackerType == trackerTypes[1])
        tracker = cv::TrackerMIL::create();
    else if (trackerType == trackerTypes[2])
        tracker = cv::TrackerKCF::create();
    else if (trackerType == trackerTypes[3])
        tracker = cv::TrackerTLD::create();
    else if (trackerType == trackerTypes[4])
        tracker = cv::TrackerMedianFlow::create();
    else if (trackerType == trackerTypes[5])
        tracker = cv::TrackerGOTURN::create();
    else if (trackerType == trackerTypes[6])
        tracker = cv::TrackerMOSSE::create();
    else {
        cout << "Incorrect tracker name" << endl;
        cout << "Available trackers are: " << endl;
        for (vector<string>::iterator it = trackerTypes.begin() ; it != trackerTypes.end(); ++it)
            std::cout << " " << *it << endl;
    }
    return tracker;
}


// Fill the vector with random colors
void getRandomColors(vector<cv::Scalar>& colors, int numColors)
{
    cv::RNG rng(0);
    for(int i=0; i < numColors; i++)
        colors.push_back(cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));
}

int main(int argc, char *argv[])
{
    std::string  names_file = "/home/maxima/cone_detection/final/data/cones.names";
    std::string  cfg_file = "/home/maxima/cone_detection/final/models/model6_small_obj_updated/yolov3-tiny_3l-cones_updated.cfg";
    std::string  weights_file = "/home/maxima/cone_detection/final/models/model6_small_obj_updated/backup/yolov3-tiny_3l-cones_updated_last.weights";
    std::string filename = "/home/maxima/cone_detection/final/data/dataset_validation/v10_0002.png";

    filename = "/home/maxima/cone_detection/final/data/videos/v10.mp4";

    if (argc > 4) {    //voc.names yolo-voc.cfg yolo-voc.weights test.mp4
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
        filename = argv[4];
    }
    else if (argc > 1) filename = argv[1];
    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.3;

    Detector detector(cfg_file, weights_file);

    auto obj_names = Detector::objects_names_from_file(names_file);


    cv::VideoCapture cap(filename);
    cv::Mat frame;

    // quit if unabke to read video file
    if(!cap.isOpened())
    {
        cout << "Error opening video file " << filename << endl;
        return -1;
    }

    // read first frame
    cap >> frame;

    vector<cv::Rect> bboxes;

    if(0){
        // Get bounding boxes for first frame
        // selectROI's default behaviour is to draw box starting from the center
        // when fromCenter is set to false, you can draw box starting from top left corner
        bool showCrosshair = true;
        bool fromCenter = false;
        cout << "\n==========================================================\n";
        cout << "OpenCV says press c to cancel objects selection process" << endl;
        cout << "It doesn't work. Press Escape to exit selection process" << endl;
        cout << "\n==========================================================\n";
        cv::selectROIs("MultiTracker", frame, bboxes, showCrosshair, fromCenter);
    }
    else{
        auto result_vec = detector.detect(frame, thresh, false);

        for(auto bbox : result_vec){
            bboxes.push_back(cv::Rect(bbox.x,bbox.y,bbox.w,bbox.h));
        }
    }


    // quit if there are no objects to track
    if(bboxes.size() < 1)
        return 0;

    vector<cv::Scalar> colors;
    getRandomColors(colors, bboxes.size());

    // Specify the tracker type
    string trackerType = trackerTypes[6];
// Create multitracker
    cv::Ptr<cv::MultiTracker> multiTracker = cv::MultiTracker::create();

// Initialize multitracker
    for(int i=0; i < bboxes.size(); i++)
        multiTracker->add(createTrackerByName(trackerType), frame, cv::Rect(bboxes[i]));


    while(cap.isOpened())
    {
        // get frame from the video
        cap >> frame;

        // Stop the program if reached end of video
        if (frame.empty()) break;

        //Update the tracking result with new frame
        multiTracker->update(frame);

        // Draw tracked objects
        for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
        {
            cv::rectangle(frame, multiTracker->getObjects()[i], colors[i], 2, 1);
        }

        // Show frame
        imshow("MultiTracker", frame);

        // quit on x button
        if  (cv::waitKey(100) == 27) break;

    }

    return 0;
}