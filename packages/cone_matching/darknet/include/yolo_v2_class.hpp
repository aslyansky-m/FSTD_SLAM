#pragma once
#ifdef LIB_EXPORTS
#if defined(_MSC_VER)
#define LIB_EXPORTS __declspec(dllexport)
#else
#define LIB_EXPORTS __attribute__((visibility("default")))
#endif
#else
#if defined(_MSC_VER)
#define LIB_EXPORTS __declspec(dllimport)
#else
#define LIB_EXPORTS
#endif
#endif

#include "Common.h"

struct image_t {
    int h;                        // height
    int w;                        // width
    int c;                        // number of chanels (3 - for RGB)
    float *data;                // pointer to the image data
};

#define C_SHARP_MAX_OBJECTS 1000
struct bbox_t_container {
    bbox_t candidates[C_SHARP_MAX_OBJECTS];
};

#ifdef __cplusplus
#include <memory>
#include <vector>
#include <deque>
#include <algorithm>

#ifdef OPENCV
#include <opencv2/opencv.hpp>            // C++
#include "opencv2/highgui/highgui_c.h"    // C
#include "opencv2/imgproc/imgproc_c.h"    // C
#endif    // OPENCV

extern "C" LIB_EXPORTS int init(const char *configurationFilename, const char *weightsFilename, int gpu);
extern "C" LIB_EXPORTS int detect_image(const char *filename, bbox_t_container &container);
extern "C" LIB_EXPORTS int detect_mat(const uint8_t* data, const size_t data_length, bbox_t_container &container);
extern "C" LIB_EXPORTS int dispose();
extern "C" LIB_EXPORTS int get_device_count();
extern "C" LIB_EXPORTS int get_device_name(int gpu, char* deviceName);

class Detector {
    std::shared_ptr<void> detector_gpu_ptr;
    std::deque<std::vector<bbox_t>> prev_bbox_vec_deque;
    const int cur_gpu_id;
public:
    float nms = .4;
    bool wait_stream;

    LIB_EXPORTS Detector(std::string cfg_filename, std::string weight_filename, int gpu_id = 0);
    LIB_EXPORTS ~Detector();

    LIB_EXPORTS std::vector<bbox_t> detect(std::string image_filename, float thresh = 0.2, bool use_mean = false);
    LIB_EXPORTS std::vector<bbox_t> detect(image_t img, float thresh = 0.2, bool use_mean = false);
    static LIB_EXPORTS image_t load_image(std::string image_filename);
    static LIB_EXPORTS void free_image(image_t m);
    LIB_EXPORTS int get_net_width() const;
    LIB_EXPORTS int get_net_height() const;
    LIB_EXPORTS int get_net_color_depth() const;

    LIB_EXPORTS std::vector<bbox_t> tracking_id(std::vector<bbox_t> cur_bbox_vec, bool const change_history = true,
                                                int const frames_story = 10, int const max_dist = 150);

    std::vector<bbox_t> detect_resized(image_t img, int init_w, int init_h, float thresh = 0.2, bool use_mean = false)
    {
        if (img.data == NULL)
            throw std::runtime_error("Image is empty");
        auto detection_boxes = detect(img, thresh, use_mean);
        float wk = (float)init_w / img.w, hk = (float)init_h / img.h;
        for (auto &i : detection_boxes) i.x *= wk, i.w *= wk, i.y *= hk, i.h *= hk;
        return detection_boxes;
    }

#ifdef OPENCV
    std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
    {
        if(mat.data == NULL)
            throw std::runtime_error("Image is empty");
        auto image_ptr = mat_to_image_resize(mat);
        return detect_resized(*image_ptr, mat.cols, mat.rows, thresh, use_mean);
    }

    std::shared_ptr<image_t> mat_to_image_resize(cv::Mat mat) const
    {
        if (mat.data == NULL) return std::shared_ptr<image_t>(NULL);

        cv::Size network_size = cv::Size(get_net_width(), get_net_height());
        cv::Mat det_mat;
        if (mat.size() != network_size)
            cv::resize(mat, det_mat, network_size);
        else
            det_mat = mat;  // only reference is copied

        return mat_to_image(det_mat);
    }

    static std::shared_ptr<image_t> mat_to_image(cv::Mat img_src)
    {
        cv::Mat img;
        cv::cvtColor(img_src, img, cv::COLOR_RGB2BGR);
        std::shared_ptr<image_t> image_ptr(new image_t, [](image_t *img) { free_image(*img); delete img; });
        std::shared_ptr<IplImage> ipl_small = std::make_shared<IplImage>(img);
        *image_ptr = ipl_to_image(ipl_small.get());
        return image_ptr;
    }

private:

    static image_t ipl_to_image(IplImage* src)
    {
        unsigned char *data = (unsigned char *)src->imageData;
        int h = src->height;
        int w = src->width;
        int c = src->nChannels;
        int step = src->widthStep;
        image_t out = make_image_custom(w, h, c);
        int count = 0;

        for (int k = 0; k < c; ++k) {
            for (int i = 0; i < h; ++i) {
                int i_step = i*step;
                for (int j = 0; j < w; ++j) {
                    out.data[count++] = data[i_step + j*c + k] / 255.;
                }
            }
        }

        return out;
    }

    static image_t make_empty_image(int w, int h, int c)
    {
        image_t out;
        out.data = 0;
        out.h = h;
        out.w = w;
        out.c = c;
        return out;
    }

    static image_t make_image_custom(int w, int h, int c)
    {
        image_t out = make_empty_image(w, h, c);
        out.data = (float *)calloc(h*w*c, sizeof(float));
        return out;
    }

#endif    // OPENCV

};



#ifdef OPENCV

static cv::Scalar obj_id_to_color(int obj_id) {
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };
    int const offset = obj_id * 123457 % 6;
    int const color_scale = 150 + (obj_id * 123457) % 100;
    cv::Scalar color(colors[offset][0], colors[offset][1], colors[offset][2]);
    color *= color_scale;
    return color;
}

class preview_boxes_t {
    enum { frames_history = 2 };    // how long to keep the history saved

    struct preview_box_track_t {
        unsigned int track_id, obj_id, last_showed_frames_ago;
        bool current_detection;
        bbox_t bbox;
        cv::Mat mat_obj, mat_resized_obj;
        preview_box_track_t() : track_id(0), obj_id(0), last_showed_frames_ago(frames_history), current_detection(false) {}
    };
    std::vector<preview_box_track_t> preview_box_track_id;
    size_t const preview_box_size, bottom_offset;
    bool const one_off_detections;
public:
    preview_boxes_t(size_t _preview_box_size = 100, size_t _bottom_offset = 100, bool _one_off_detections = false) :
        preview_box_size(_preview_box_size), bottom_offset(_bottom_offset), one_off_detections(_one_off_detections)
    {}

    void set(cv::Mat src_mat, std::vector<bbox_t> result_vec)
    {
        size_t const count_preview_boxes = src_mat.cols / preview_box_size;
        if (preview_box_track_id.size() != count_preview_boxes) preview_box_track_id.resize(count_preview_boxes);

        // increment frames history
        for (auto &i : preview_box_track_id)
            i.last_showed_frames_ago = std::min((unsigned)frames_history, i.last_showed_frames_ago + 1);

        // occupy empty boxes
        for (auto &k : result_vec) {
            bool found = false;
            // find the same (track_id)
            for (auto &i : preview_box_track_id) {
                if (i.track_id == k.track_id) {
                    if (!one_off_detections) i.last_showed_frames_ago = 0; // for tracked objects
                    found = true;
                    break;
                }
            }
            if (!found) {
                // find empty box
                for (auto &i : preview_box_track_id) {
                    if (i.last_showed_frames_ago == frames_history) {
                        if (!one_off_detections && k.frames_counter == 0) break; // don't show if obj isn't tracked yet
                        i.track_id = k.track_id;
                        i.obj_id = k.obj_id;
                        i.bbox = k;
                        i.last_showed_frames_ago = 0;
                        break;
                    }
                }
            }
        }

        // draw preview box (from old or current frame)
        for (size_t i = 0; i < preview_box_track_id.size(); ++i)
        {
            // get object image
            cv::Mat dst = preview_box_track_id[i].mat_resized_obj;
            preview_box_track_id[i].current_detection = false;

            for (auto &k : result_vec) {
                if (preview_box_track_id[i].track_id == k.track_id) {
                    if (one_off_detections && preview_box_track_id[i].last_showed_frames_ago > 0) {
                        preview_box_track_id[i].last_showed_frames_ago = frames_history; break;
                    }
                    bbox_t b = k;
                    cv::Rect r(b.x, b.y, b.w, b.h);
                    cv::Rect img_rect(cv::Point2i(0, 0), src_mat.size());
                    cv::Rect rect_roi = r & img_rect;
                    if (rect_roi.width > 1 || rect_roi.height > 1) {
                        cv::Mat roi = src_mat(rect_roi);
                        cv::resize(roi, dst, cv::Size(preview_box_size, preview_box_size), cv::INTER_NEAREST);
                        preview_box_track_id[i].mat_obj = roi.clone();
                        preview_box_track_id[i].mat_resized_obj = dst.clone();
                        preview_box_track_id[i].current_detection = true;
                        preview_box_track_id[i].bbox = k;
                    }
                    break;
                }
            }
        }
    }


    void draw(cv::Mat draw_mat, bool show_small_boxes = false)
    {
        // draw preview box (from old or current frame)
        for (size_t i = 0; i < preview_box_track_id.size(); ++i)
        {
            auto &prev_box = preview_box_track_id[i];

            // draw object image
            cv::Mat dst = prev_box.mat_resized_obj;
            if (prev_box.last_showed_frames_ago < frames_history &&
                dst.size() == cv::Size(preview_box_size, preview_box_size))
            {
                cv::Rect dst_rect_roi(cv::Point2i(i * preview_box_size, draw_mat.rows - bottom_offset), dst.size());
                cv::Mat dst_roi = draw_mat(dst_rect_roi);
                dst.copyTo(dst_roi);

                cv::Scalar color = obj_id_to_color(prev_box.obj_id);
                int thickness = (prev_box.current_detection) ? 5 : 1;
                cv::rectangle(draw_mat, dst_rect_roi, color, thickness);

                unsigned int const track_id = prev_box.track_id;
                std::string track_id_str = (track_id > 0) ? std::to_string(track_id) : "";
                putText(draw_mat, track_id_str, dst_rect_roi.tl() - cv::Point2i(-4, 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.9, cv::Scalar(0, 0, 0), 2);

                std::string size_str = std::to_string(prev_box.bbox.w) + "x" + std::to_string(prev_box.bbox.h);
                putText(draw_mat, size_str, dst_rect_roi.tl() + cv::Point2i(0, 12), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);

                if (!one_off_detections && prev_box.current_detection) {
                    cv::line(draw_mat, dst_rect_roi.tl() + cv::Point2i(preview_box_size, 0),
                        cv::Point2i(prev_box.bbox.x, prev_box.bbox.y + prev_box.bbox.h),
                        color);
                }

                if (one_off_detections && show_small_boxes) {
                    cv::Rect src_rect_roi(cv::Point2i(prev_box.bbox.x, prev_box.bbox.y),
                        cv::Size(prev_box.bbox.w, prev_box.bbox.h));
                    unsigned int const color_history = (255 * prev_box.last_showed_frames_ago) / frames_history;
                    color = cv::Scalar(255 - 3 * color_history, 255 - 2 * color_history, 255 - 1 * color_history);
                    if (prev_box.mat_obj.size() == src_rect_roi.size()) {
                        prev_box.mat_obj.copyTo(draw_mat(src_rect_roi));
                    }
                    cv::rectangle(draw_mat, src_rect_roi, color, thickness);
                    putText(draw_mat, track_id_str, src_rect_roi.tl() - cv::Point2i(0, 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
                }
            }
        }
    }
};
#endif    // OPENCV

//extern "C" {
#endif    // __cplusplus

