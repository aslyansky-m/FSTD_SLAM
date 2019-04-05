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

#include "ORBextractor.h"
#include "KLTtracker.h"

#include <algorithm>



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


int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        auto v = (unsigned int)(*pa ^ *pb);
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}


float MARGIN = 0.8;

class Object{
public:
    int id;
    int age;
    bbox_t bbox;
    cv::KeyPoint keypoint;
    cv::Mat descriptor;
    cv::Mat resized_patch;
};

class Tracker{
public:
    class Params{
    public:
        int frame_history = 1;
        int draw_size = 60;
        bool draw_lines = false;
        static constexpr int num_classes = 3;
        //                                     yellow       blue       orange
         int colors[num_classes][3] ={{0,255,255},{255,255,255},{0,125,255} };
    };

    Params params;
    int tot_num = 0;
    int cur_time = 0;

    std::vector<Object> objects;



    void update(){
        cur_time++;
    }

//    void draw(cv::Mat draw_mat){
//
//        int displayed[num_classes] = {0};
//
//        int step = params.draw_size;
//        cv::Size2i plot_size(step, step);
//
//
//        for(auto object : objects){
//            int id = object.bbox.obj_id;
//            if(displayed[id] > draw_num) continue;
//            int offset_x = (displayed[id]++)*step;
//            int offset_y = (id + 1)*step;
//            cv::Scalar color = params.colors[id];
//
//            cv::Rect roi({offset_x, draw_mat - offset_y},plot_size);
//            cv::Mat patch_roi = draw_mat(roi);
//            cv::resize(object.resized_patch, patch_roi, plot_size);
//
//            cv::line(draw_mat, roi.tl() + cv::Point2i(step/2, 0), cv::Point2i(object.bbox.x, object.bbox.y + object.bbox.h/2), color);
//
//        }
//
//    }

    void draw(cv::Mat draw_mat, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names,
              int current_det_fps = -1){

        int displayed[params.num_classes] = {0};

        int step = params.draw_size;
        cv::Size2i plot_size(step, step);

        cv::Mat aux;

        cv::Mat orig = draw_mat.clone();
        cv::Rect img_rect(cv::Point2i(0, 0), orig.size());

        float margin = MARGIN;

        int num=0;
        for(auto object : result_vec){
            int id = object.obj_id;
            if(id >= params.num_classes) continue;
            int offset_x = displayed[id]*step;
            if(offset_x + step > orig.cols) continue;
            int offset_y = (id + 1)*step;
            cv::Scalar color(params.colors[id][0],params.colors[id][1],params.colors[id][2]);

            cv::Rect rect_roi = cv::Rect(object.x - margin*object.w, object.y - margin*object.h, object.w + 2*margin*object.w, object.h + 2*margin*object.h) & img_rect;
            cv::rectangle(draw_mat, rect_roi, color, 2);

            if (rect_roi.width <= 0 && rect_roi.height <= 0) continue;

            std::string label = std::to_string(++num);
            cv::putText(draw_mat, label, cv::Point2f(object.x + 0.5f*object.w, object.y - margin*object.h + 10), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0), 2);

            cv::Mat roi = orig(rect_roi);
            cv::resize(roi, aux, plot_size, cv::INTER_LINEAR);
            cv::Rect plot_rect({offset_x, draw_mat.rows - offset_y},plot_size);
            cv::Mat plot_roi = draw_mat(plot_rect);
            aux.copyTo(plot_roi);
            cv::rectangle(draw_mat, plot_rect, color, 2);
            if(params.draw_lines)
                cv::line(draw_mat, plot_rect.tl() + cv::Point2i(step/2, 0), cv::Point2i(object.x, object.y + object.h/2), color);
            displayed[id]++;
        }

    }


};







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



    Tracker t;
    ORB_SLAM2::ORBextractor extractor(100,MARGIN, false);


    KLTtracker tracker(KLTtracker::Params);


    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.8;

    Detector detector(cfg_file, weights_file);

    auto obj_names = objects_names_from_file(names_file);
    std::string out_videofile = "/home/maxima/Desktop/result.avi";
    bool save_output_videofile = false;
    cv::VideoWriter output_video;
    if (save_output_videofile){}
        //output_video.open(out_videofile, CV_FOURCC('D', 'I', 'V', 'X'), 25, frame_size, true);

    while (true)
    {
        std::cout << "input image or video filename: ";
        if(filename.size() == 0) std::cin >> filename;
        if (filename.size() == 0) break;

        try {

            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;


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
                detector.nms = 0.4;    // comment it - if track_id is not required
                bool exit_flag = false;
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

                cv::Mat show_frame, gray_frame;
                cv::Size show_size(1920,frame_size.height*1920/frame_size.width);
                steady_start = std::chrono::steady_clock::now();

                std::vector<int> frames = {500,503};
                struct frame_data {
                    std::vector<bbox_t> bboxes;
                    std::vector<cv::KeyPoint> keypoints;
                    cv::Mat descriptors;
                    std::vector<cv::Rect> rects;
                    std::vector<cv::Mat> objects;
                    cv::Mat image;
                    cv::Mat viz;

                };



                std::vector<frame_data> frame_datas;

                std::vector<cv::Rect> rects;
                std::vector<cv::Mat> images;
                int nframe = 0;
                while (!cur_frame.empty())
                {
                    nframe++;

                    cap >> cap_frame;
                    if(nframe < frames[0] - 1) continue;

                    {
                        steady_end = std::chrono::steady_clock::now();
                        if (std::chrono::duration<double>(steady_end - steady_start).count() >= 1) {
                            current_det_fps = fps_det_counter;
                            steady_start = steady_end;
                            fps_det_counter = 0;
                        }
                        fps_det_counter++;
                    }


                    cur_frame = cap_frame.clone();
                    det_image = detector.mat_to_image_resize(cap_frame);
                    auto current_image = det_image;
                    thread_result_vec = detector.detect_resized(*current_image, frame_size.width, frame_size.height, thresh, false);

                    cv::cvtColor(cur_frame,gray_frame,cv::COLOR_BGR2GRAY);
                    extractor(gray_frame,thread_result_vec,keypoints,descriptors,rects,images);

                    t.draw(cur_frame, thread_result_vec, obj_names, current_det_fps);
                    cv::resize(cur_frame,show_frame,show_size,cv::INTER_LINEAR);
                    cv::imshow("window name", show_frame);
                    int key = cv::waitKey(10);    // 3 or 16ms
                    if (key == 'f') show_small_boxes = !show_small_boxes;
                    if (key == 'p') while (true) if(cv::waitKey(100) == 'p') break;
                    if (key == 'e') extrapolate_flag = !extrapolate_flag;
                    if (key == 27) { exit_flag = true; break; }

                    if(std::find(frames.begin(),frames.end(),nframe) != frames.end()){
                        frame_data data;
                        data.bboxes = thread_result_vec;
                        data.objects = std::move(images);
                        data.image = cap_frame.clone();
                        data.rects = std::move(rects);
                        data.keypoints = keypoints;
                        data.descriptors = descriptors.clone();
                        data.viz = cur_frame.clone();
                        frame_datas.push_back(data);
                        if(frame_datas.size()==frames.size())
                            break;
                    }
                }
                cv::destroyAllWindows();

                //std::vector<cv::Point2f> prev_points, next_points;cv::Mat status;

                for (int i = 0; i < frame_datas.size(); i++){
                    cv::imshow(std::to_string(frames[i]),frame_datas[i].viz);
                    cv::moveWindow(std::to_string(frames[i]), i*960+20,100);
                }

                auto &desc1 = frame_datas[0].descriptors;
                auto &desc2 = frame_datas[1].descriptors;
                int n1 = desc1.rows, n2 = desc2.rows;
                cv::Mat dist_mat(n1,n2,CV_32SC1);

                std::vector<int> match(n1);

                for (int i = 0; i<n1; i++)
                    for (int j = 0; j<n2; j++)
                    {


                        int dist = DescriptorDistance(desc1.row(i),desc2.row(j));

                        dist_mat.at<int>(i,j) = dist;

                    }

                std::cout << '\n' << dist_mat;

                while (true) if(cv::waitKey(100) == 27) break;

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