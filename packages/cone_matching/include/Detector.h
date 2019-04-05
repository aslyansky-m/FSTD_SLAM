//
// Created by maxima on 26/01/19.
//

#pragma once

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

    static std::vector<std::string> objects_names_from_file(std::string const filename) {
        std::ifstream file(filename);
        std::vector<std::string> file_lines;
        if (!file.is_open()) return file_lines;
        for(std::string line; getline(file, line);) file_lines.push_back(line);
        std::cout << "object names loaded \n";
        return file_lines;
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


//extern "C" {
#endif    // __cplusplus


