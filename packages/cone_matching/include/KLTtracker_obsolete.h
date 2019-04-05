//
// Created by maxima on 26/01/19.
//


//
//class TrackVisualizer(){
//cv::Mat mask;
//    class Params{
//        double display_motion_scale = 10.0;
//        double display_scale = 1.0;
//        double display_fade_factor = 0.4;
//    };
//
//    std::vector<cv::Scalar> colors;
//
//    void draw(cv::Mat &image, const std::vector<cv::Point2f> &prev_points, const std::vector<cv::Point2f> &next_points) {
//        mask *= params.display_fade_factor;
//        for (int i = 0; i < next_points.size(); i++) {
//            cv::Scalar color = motion_to_hsv(next_points[i] - prev_points[i]);
//            cv::line(mask, prev_points[i], next_points[i], color, 4*params.display_scale+0.5);
//            cv::circle(image, next_points[i], 4*params.display_scale+0.5, color, -1);
//        }
//        cv::add(image, mask, image);
//    }
//
//    std::vector<std::string> stats(int num_points, int removed, int added) {
//        std::vector<std::string> str_vec;
//        std::string text;
//        text = "alive: " + std::to_string(num_points);
//        str_vec.push_back(text);
//        text = "removed: " + std::to_string(removed);
//        str_vec.push_back(text);
//        text = "added: " + std::to_string(added);
//        str_vec.push_back(text);
//        return str_vec;
//    }
//
//protected:
//    static std::vector<uchar> mat_to_vec(cv::Mat &mat) {
//        std::vector<uchar> array;
//        if (mat.isContinuous()) {
//            array.assign(mat.datastart, mat.dataend);
//        }
//        else {
//            for (int i = 0; i < mat.rows; ++i) {
//                array.insert(array.end(), mat.ptr<uchar>(i), mat.ptr<uchar>(i) + mat.cols);
//            }
//        }
//        return array;
//    }
//}
//
//
//
//class KLTtrackerVis : public KLTtracker
//{
//public:
//    int num_points = 0, removed = 0, added = 0;
//    std::vector<cv::Point2f> m_prevPts, m_nextPts;
//public:
//    KLT() {}
//    KLT(cv::Size image_size, Params params, KLT_base::Params params)  : KLT_base(image_size,params), params(params) {}
//
//    bool track(const cv::Mat &image) {
//        m_prevPts = m_nextPts;
//        std::vector<uchar> valid;
//        bool result = KLT_base::track(image,m_prevPts,m_nextPts,valid);
//        if(result)
//            remove_invalid(valid);
//        return result;
//    }
//
//    void add_points(const std::vector<cv::Point2f> &newPoints) {
//        int init_size = num_points;
//        added = newPoints.size();
//        num_points += added;
//        m_prevPts.resize(num_points);
//        m_nextPts.resize(num_points);
//        colors.resize(num_points);
//
//        cv::RNG rng;
//        for (int i = init_size, j = 0; i < num_points; i++, j++) {
//            m_nextPts[i] = newPoints[j];
//            colors[i] = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//        }
//    }
//
//    cv::Mat get_mask() {
//        return KLT_base::get_mask(m_nextPts);
//    }
//
//    int max_add() {
//        if (is_first)
//            return params.max_pts_tot;
//        int pts_to_add = MAX(MAX(0, MIN(params.max_pts_add, params.max_pts_tot - num_points)), (params.min_pts_tot - num_points));
//        return pts_to_add;
//    }
//
//    void draw(cv::Mat &image) {
//        KLT_base::draw(image, m_prevPts, m_nextPts);
//    }
//
//    std::vector<std::string> stats() {
//        return KLT_base::stats(num_points, removed, added);
//    }
//
//protected:
//    void remove_invalid(const std::vector<uchar> &valid) {
//        int last = 0;
//        int valid_size = valid.size();
//        for (int i = 0; i<valid_size; ++i, ++last){
//            while (!valid[i]) {
//                ++i;
//                if (i >= valid_size)
//                    break;
//            }
//            if (i >= valid_size)
//                break;
//            m_prevPts[last] = m_prevPts[i];
//            m_nextPts[last] = m_nextPts[i];
//            colors[last] = colors[i];
//        }
//
//        m_prevPts.resize(last);
//        m_nextPts.resize(last);
//        colors.resize(last);
//        removed = num_points - last;
//        num_points = last;
//    }
//
//    inline cv::Scalar motion_to_hsv(cv::Point2f motion) {
//        float ang = fatan2(motion.y, motion.x) * 180 / CV_PI / 2;
//        if (ang < 0)
//            ang = 180 + ang;
//        float mag = MAX(0.0f, MIN(1.0f, (float)sqrt(motion.y*motion.y + motion.x*motion.x) / params.display_motion_scale));
//        cv::Scalar hsv(ang, 255, 255 * mag);
//        return hsv_to_rgb(hsv);
//    }
//
//    inline cv::Scalar hsv_to_rgb(cv::Scalar &hsv) {
//        cv::Mat bgr_mat;
//        cv::Mat hsv_mat(1, 1, CV_8UC3, hsv);
//        cv::cvtColor(hsv_mat, bgr_mat, cv::COLOR_HSV2BGR);
//        return cv::Scalar(bgr_mat.data[0], bgr_mat.data[1], bgr_mat.data[2]);
//    }
//
//    inline cv::Scalar motion_to_bgr(cv::Point2f motion) {
//        float mag = MAX(0.0f, MIN(1.0f, (float)sqrt(motion.y*motion.y + motion.x*motion.x) / params.display_motion_scale));
//        cv::Scalar color = cv::Scalar((1 - mag) * 255, 0, mag * 255);
//        return color;
//    }
//};



