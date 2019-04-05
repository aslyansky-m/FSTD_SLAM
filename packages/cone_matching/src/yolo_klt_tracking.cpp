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

float MARGIN = 0.0;

class Object{
public:
    int born;
    int last_detected;
    int num_detected;
    float prob;
    int obj_id;
    int class_id;
    cv::Point2f point;
    cv::Rect2f rect;
    cv::Mat patch;

    int age(int time) const {return time - born;} ;
    int seen_ago(int time) const {return time - last_detected;};
};

template <class T, typename P>
static void erase_if(std::vector<T> vec, P lambda){
    vec.erase(std::remove_if(vec.begin(), vec.end(),lambda,vec.end()));
}

class Tracker{
public:
    class Params{
    public:
        int max_frames_blind = 5;
        float track_tresh = 200.0;
        int draw_size = 60;
        int patch_size = 25;
        bool draw_lines = true;
        static constexpr int num_classes = 3;
        int colors[num_classes][3] ={{0,255,255},{255,255,255},{0,125,255}}; // yellow,blue,orange
    };

    Params params;
    int num_objects = 0;
    int num_frame = 0;

    bool is_first = true;

    KLTtracker klt = KLTtracker(KLTtracker::Params(10,2));

    std::vector<Object> objects;
    std::vector<std::string> obj_names;

    std::vector<cv::Point2f> prev_points, next_points;
    std::vector<uchar> status;
    cv::Mat gray_image;

    void update(cv::Mat image, const std::vector<bbox_t> detections){

        cv::Rect img_rect(cv::Point2i(0, 0), image.size());
        cv::Size2i patch_size(params.patch_size, params.patch_size);

        if(is_first){
            objects.clear(); objects.reserve(detections.size());
            prev_points.clear();  prev_points.reserve(detections.size());

            for(auto bbox : detections){
                Object obj;
                obj.rect = cv::Rect(bbox.x, bbox.y, bbox.w, bbox.h) & img_rect;
                obj.class_id = bbox.obj_id;
                if(obj.class_id >= params.num_classes) continue;
                if (obj.rect.width == 0 || obj.rect.height == 0) continue;
                obj.point = {bbox.x + bbox.w*0.5f,bbox.y + bbox.h*0.5f};
                obj.prob = bbox.prob;
                obj.obj_id = num_objects++;
                obj.born = num_frame;
                obj.last_detected = num_frame;
                obj.num_detected = 1;
                cv::resize(image(obj.rect),obj.patch,patch_size);
                objects.push_back(obj);
                prev_points.push_back(obj.point);
            }
        }
        else{
            // remove too old objects that weren't updated
            int erased = objects.size();
            objects.erase(std::remove_if(objects.begin(), objects.end(),[&](const Object & o) {
                        return o.seen_ago(num_frame) > params.max_frames_blind;
                    }),objects.end());
            erased -= objects.size();
            //std::cout << erased << "\n";
            // get points from objects for tracking
            prev_points.clear();
            prev_points.reserve(objects.size());
            std::transform(objects.begin(),objects.end(),std::back_inserter(prev_points),[](const Object &o) -> cv::Point2f {
                return o.point;
            });
        }

        cv::cvtColor(image,gray_image,cv::COLOR_BGR2GRAY);
        bool ret = klt.operator()(gray_image,prev_points,next_points,status);

        if(ret){
            // update existing objects
            int cur = 0;
            for(int i = 0 ; i < objects.size(); i++){
                if(status[i]){
                    objects[i].point = next_points[i];
                    objects[i].rect.x = next_points[i].x;
                    objects[i].rect.y = next_points[i].y;
                    objects[cur++] = std::move(objects[i]);
                }
            }
            objects.resize(cur);

            // associate to detected ones
            for (auto bbox : detections){

                if( bbox.obj_id >= params.num_classes) continue;
                cv::Rect rect(bbox.x, bbox.y, bbox.w, bbox.h);
                rect = rect & img_rect;
                if (rect.width == 0 || rect.height == 0) continue;

                cv::Point2f center = {bbox.x + bbox.w*0.5f,bbox.y + bbox.h*0.5f};
                Object* closest = nullptr; float min_distance = 1e10f;
                // find closest
                for(auto &object : objects){
                    // continue if found neighbor
                    if(object.seen_ago(num_frame)==0) continue;
                    // else search for the closest
                    cv::Point2f diff = center - object.point;
                    float cur_distance = diff.x*diff.x+diff.y*diff.y;
                    if(cur_distance < min_distance){
                        min_distance = cur_distance;
                        closest = &object;
                    }
                }
                // if found close enough update the object
                if(closest &&
                    (min_distance < params.track_tresh ||
                    (center.x > closest->rect.x && center.x < closest->rect.x + closest->rect.width &&
                     center.y > closest->rect.y && center.y < closest->rect.y + closest->rect.height))
                ){
                    Object& obj = *closest;
                    obj.rect = rect;
                    obj.class_id = bbox.obj_id;
                    obj.point = {bbox.x + bbox.w*0.5f,bbox.y + bbox.h*0.5f};
                    obj.prob = bbox.prob;
                    obj.num_detected++;
                    obj.last_detected = num_frame;
                    cv::resize(image(obj.rect),obj.patch,patch_size);
                }
                // otherwise create a new one
                else{
                    Object obj;
                    obj.rect = cv::Rect(bbox.x, bbox.y, bbox.w, bbox.h) & img_rect;
                    obj.class_id = bbox.obj_id;
                    obj.point = {bbox.x + bbox.w*0.5f,bbox.y + bbox.h*0.5f};
                    obj.prob = bbox.prob;
                    obj.obj_id = num_objects++;
                    obj.born = num_frame;
                    obj.last_detected = num_frame;
                    obj.num_detected = 1;
                    cv::resize(image(obj.rect),obj.patch,patch_size);
                    objects.push_back(obj);
                }

            }

        }

        is_first = false;
        num_frame++;
    }

    void draw(cv::Mat draw_mat){

        int displayed[params.num_classes] = {0};

        int step = params.draw_size;
        cv::Size2i plot_size(step, step);

        cv::Mat aux;

        cv::Mat orig = draw_mat.clone();

        float margin = MARGIN;


        if(1){
            std::sort(objects.begin(),objects.end(),[](Object a, Object b) {
                return a.point.x < b.point.x;
            });
        }
        else{
            std::sort(objects.begin(),objects.end(),[](Object a, Object b) {
                return a.obj_id < b.obj_id;
            });
        }


        int num=0;
        for(auto object : objects){
            int id = object.class_id;
            int offset_x = displayed[id]*step;

            int offset_y = (id + 1)*step;

            float A = (50 + object.obj_id%50)*0.01f;

            cv::Scalar color(params.colors[id][0] + 120*A,params.colors[id][1],params.colors[id][2]);

            if (object.seen_ago(num_frame)>1)
                color = {0,0,255};

            cv::Rect rect_roi = object.rect;
            cv::rectangle(draw_mat, rect_roi, color, 2);

            std::string label = std::to_string(object.obj_id);
            cv::putText(draw_mat, label, object.rect.tl() + cv::Point2f(5, 15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);


            if(offset_x + step > orig.cols) continue;

            cv::resize(object.patch, aux, plot_size, cv::INTER_LINEAR);
            cv::Rect plot_rect({offset_x, draw_mat.rows - offset_y},plot_size);
            cv::Mat plot_roi = draw_mat(plot_rect);
            aux.copyTo(plot_roi);
            cv::rectangle(draw_mat, plot_rect, color, 2);
            if(params.draw_lines)
                cv::line(draw_mat, plot_rect.tl() + cv::Point2i(step/2, 0), object.point + cv::Point2f(0, object.rect.height/2), color);

            cv::putText(draw_mat, label, plot_rect.tl() + cv::Point2i(step/4, step/4), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
            displayed[id]++;
        }

    }


};


int main(int argc, char *argv[])
{
    std::string  names_file = "/home/maxima/cone_detection/final/data/cones.names";
    std::string  cfg_file = "/home/maxima/cone_detection/final/models/model6_small_obj_updated/yolov3-tiny_3l-cones_updated.cfg";
    std::string  weights_file = "/home/maxima/cone_detection/final/models/model6_small_obj_updated/backup/yolov3-tiny_3l-cones_updated_last.weights";
    std::string filename = "/home/maxima/cone_detection/final/data/dataset_validation/v10_0002.png";

    filename = "/home/maxima/cone_detection/final/data/videos/v12_ds.mp4";

    if (argc > 4) {    //voc.names yolo-voc.cfg yolo-voc.weights test.mp4
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
        filename = argv[4];
    }
    else if (argc > 1) filename = argv[1];    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.1;

    Detector detector(cfg_file, weights_file);

    auto obj_names = Detector::objects_names_from_file(names_file);


    Tracker tracker;

    std::string const file_ext = filename.substr(filename.find_last_of(".") + 1);
    std::string const protocol = filename.substr(0, 7);

    cv::Mat draw_frame, cur_frame, det_frame, write_frame;
    std::vector<bbox_t> result_vec;
    detector.nms = 0.05;

    cv::VideoCapture cap(filename); cap >> cur_frame;
    cv::Size const frame_size = cur_frame.size();

    cv::Mat show_frame, gray_frame;
    cv::Size show_size(1920,frame_size.height*1920/frame_size.width);

    int const video_fps = cap.get(CV_CAP_PROP_FPS);
    std::string out_videofile = "/home/maxima/Desktop/result.avi";
    bool save_output_videofile = false;
    cv::VideoWriter output_video;
    if (save_output_videofile)
        output_video.open(out_videofile, CV_FOURCC('D', 'I', 'V', 'X'), std::max(35, video_fps), frame_size, true);


    int nframe = 0;
    while (true)
    {
        nframe++;

        cap >> cur_frame;
        if(cur_frame.empty()) break;
        result_vec = detector.detect(cur_frame, thresh, false);

        //draw
        draw_frame = cur_frame.clone();
        tracker.update(cur_frame,result_vec);
        tracker.draw(draw_frame);

        cv::resize(draw_frame,show_frame, show_size,cv::INTER_LINEAR);
        cv::imshow("window name", show_frame);

        //wait
        int key = cv::waitKey(1);    // 3 or 16ms
        if (key == 'p') while (true) if(cv::waitKey(100) == 'p') break;
        if (key == 27) { break; }

        if(save_output_videofile) {
            output_video << draw_frame;
        }

    }
    cv::destroyAllWindows();
    std::cout << "Video ended \n";
    std::cout << tracker.num_objects << std::endl;


    filename.clear();


    return 0;
}