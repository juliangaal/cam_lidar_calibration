#ifndef load_params_h_
#define load_params_h_

#include <ros/ros.h>
#include <opencv2/core/mat.hpp>

#include "node_handle.h"

namespace cam_lidar_calibration
{

    struct initial_parameters_t
    {
        initial_parameters_t()
        : fisheye_model(false)
        , lidar_ring_count(0)
        {}
        ~initial_parameters_t() = default;

        bool fisheye_model;
        int lidar_ring_count;
        cv::Size chessboard_pattern_size;
        int square_length;                 // in millimetres
        cv::Size board_dimensions;         // in millimetres
        cv::Point3d cb_translation_error;  // in millimetres
        cv::Mat cameramat, distcoeff;
        std::pair<int, int> image_size;  // in pixels
        std::string camera_topic, camera_info, lidar_topic;
    };

    struct fixed_parameters_t
    {
        fixed_parameters_t() = default;
        ~fixed_parameters_t() = default;
        fs::path import_path;
        bool import_samples;
        int queue_rate;
        size_t num_lowestvoq;
        double distance_offset;
    };

    void loadParams(const ros::NodeHandle& n, initial_parameters_t& i_params);

    void loadParams(const CustomNodeHandle& n, fixed_parameters_t& f_params);

}  // namespace cam_lidar_calibration

#endif
