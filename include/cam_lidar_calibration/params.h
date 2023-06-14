#ifndef load_params_h_
#define load_params_h_

#include <ros/ros.h>
#include <opencv2/core/mat.hpp>

#include "node_handle.h"

namespace cam_lidar_calibration
{

    struct initial_parameters_t
    {
        initial_parameters_t() = default;
        ~initial_parameters_t() = default;

        bool fisheye_model {false};
        int lidar_ring_count {0};
        int square_length {0};
        cv::Mat cameramat = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat distcoeff = cv::Mat::eye(1, 4, CV_64F);
        
        cv::Size chessboard_pattern_size;
        std::string camera_topic;
        std::string camera_info;
        std::string lidar_topic;
        cv::Size board_dimensions;         // in millimetres
        cv::Point3d cb_translation_error;  // in millimetres
        std::pair<int, int> image_size;  // in pixels
        
        void load(const ros::NodeHandle& n);
    };

    struct fixed_parameters_t
    {
        fixed_parameters_t() = default;
        ~fixed_parameters_t() = default;
        
        bool import_samples {false};
        int queue_rate {20};
        size_t num_lowestvoq {0};
        double distance_offset {0};
        fs::path import_path;
        void load(const CustomNodeHandle& n);
    };
}  // namespace cam_lidar_calibration

#endif
