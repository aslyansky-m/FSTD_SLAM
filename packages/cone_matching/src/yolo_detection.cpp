#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

//#include "ORBextractor.h"

//ORB_SLAM2::ORBextractor extractor(100,1.2,5,20,4);


// To use tracking - uncomment the following line. Tracking is supported only by OpenCV 3.x
//#define TRACK_OPTFLOW

//#include "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v9.1\include\cuda_runtime.h"
//#pragma comment(lib, "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v9.1/lib/x64/cudart.lib")
//static std::shared_ptr<image_t> device_ptr(NULL, [](void *img) { cudaDeviceReset(); });

#include "yolo_v2_class.hpp"    // imported functions from DLL

#include <opencv2/opencv.hpp>            // C++
#include "opencv2/core/version.hpp"
#ifndef CV_VERSION_EPOCH
#include "opencv2/videoio/videoio.hpp"
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")




void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, 
    int current_det_fps = -1)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int const max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 30, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols-1), std::min((int)i.y, mat_img.rows-1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
        }
    }
    if (current_det_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }
}
#endif    // OPENCV


void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names) {
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
                  << ", w = " << i.w << ", h = " << i.h
                  << std::setprecision(3) << ", prob = " << i.prob << std::endl;
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
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



    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.20;

    Detector detector(cfg_file, weights_file);

    auto obj_names = objects_names_from_file(names_file);
    std::string out_videofile = "result.avi";
    bool const save_output_videofile = true;

    while (true)
    {
        std::cout << "input image or video filename: ";
        if(filename.size() == 0) std::cin >> filename;
        if (filename.size() == 0) break;

        try {
            bool extrapolate_flag = false;
            float cur_time_extrapolate = 0, old_time_extrapolate = 0;
            preview_boxes_t large_preview(100, 150, false), small_preview(50, 50, true);
            bool show_small_boxes = false;

            std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
            std::string const protocol = filename.substr(0, 7);
            if (file_ext == "avi" || file_ext == "mp4" || file_ext == "mjpg" || file_ext == "mov" ||     // video file
                protocol == "rtmp://" || protocol == "rtsp://" || protocol == "http://" || protocol == "https:/")    // video network stream
            {
                cv::Mat cap_frame, cur_frame, det_frame, write_frame;
                std::queue<cv::Mat> track_optflow_queue;
                int passed_flow_frames = 0;
                std::shared_ptr<image_t> det_image;
                std::vector<bbox_t> result_vec, thread_result_vec;
                detector.nms = 0.02;    // comment it - if track_id is not required
                std::atomic<bool> consumed, videowrite_ready;
                bool exit_flag = false;
                consumed = true;
                videowrite_ready = true;
                std::atomic<int> fps_det_counter, fps_cap_counter;
                fps_det_counter = 0;
                fps_cap_counter = 0;
                int current_det_fps = 0;
                std::thread t_detect, t_cap, t_videowrite;
                std::mutex mtx;
                std::condition_variable cv_detected, cv_pre_tracked;
                std::chrono::steady_clock::time_point steady_start, steady_end;
                cv::VideoCapture cap(filename); cap >> cur_frame;
                int const video_fps = cap.get(CV_CAP_PROP_FPS);
                cv::Size const frame_size = cur_frame.size();
                cv::VideoWriter output_video;
                if (save_output_videofile)
                    output_video.open(out_videofile, CV_FOURCC('D', 'I', 'V', 'X'), std::max(35, video_fps), frame_size, true);

                steady_start = std::chrono::steady_clock::now();

                while (!cur_frame.empty())
                {
                    {
                        steady_end = std::chrono::steady_clock::now();
                        if (std::chrono::duration<double>(steady_end - steady_start).count() >= 1) {
                            current_det_fps = fps_det_counter;
                            steady_start = steady_end;
                            fps_det_counter = 0;
                        }
                        fps_det_counter++;
                    }

                    cap >> cap_frame;
                    cur_frame = cap_frame.clone();
                    det_image = detector.mat_to_image_resize(cap_frame);
                    auto current_image = det_image;
                    thread_result_vec = detector.detect_resized(*current_image, frame_size.width, frame_size.height, thresh, false);
                    auto tmp_result_vec = detector.tracking_id(thread_result_vec, false);
                    small_preview.set(cur_frame, tmp_result_vec);
                    large_preview.set(cur_frame, tmp_result_vec);
                    draw_boxes(cur_frame, thread_result_vec, obj_names, current_det_fps);
                    small_preview.draw(cur_frame, show_small_boxes);
//                    large_preview.draw(cur_frame);
                    cv::imshow("window name", cur_frame);
                    int key = cv::waitKey(3);    // 3 or 16ms
                    if (key == 'f') show_small_boxes = !show_small_boxes;
                    if (key == 'p') while (true) if(cv::waitKey(100) == 'p') break;
                    if (key == 'e') extrapolate_flag = !extrapolate_flag;
                    if (key == 27) { exit_flag = true; break; }
                }

                std::cout << "Video ended \n";
                break;
            }
        }
        catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
        catch (...) { std::cerr << "unknown exception \n"; getchar(); }
        filename.clear();
    }

    return 0;
}