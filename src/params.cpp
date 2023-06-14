#include "cam_lidar_calibration/params.h"

namespace cam_lidar_calibration
{
    void initial_parameters_t::load(const ros::NodeHandle& n)
    {
        int cb_w, cb_h, w, h, e_x, e_y;
        ROS_ASSERT(n.getParam("camera_topic", camera_topic));
        ROS_ASSERT(n.getParam("camera_info", camera_info));
        ROS_ASSERT(n.getParam("lidar_topic", lidar_topic));
        ROS_ASSERT(n.getParam("chessboard/pattern_size/width", cb_w));
        ROS_ASSERT(n.getParam("chessboard/pattern_size/height", cb_h));
        chessboard_pattern_size = cv::Size(cb_w, cb_h);
        ROS_ASSERT(n.getParam("chessboard/square_length", square_length));
        ROS_ASSERT(n.getParam("chessboard/board_dimension/width", w));
        ROS_ASSERT(n.getParam("chessboard/board_dimension/height", h));
        board_dimensions = cv::Size(w, h);
        ROS_ASSERT(n.getParam("chessboard/translation_error/x", e_x));
        ROS_ASSERT(n.getParam("chessboard/translation_error/y", e_y));
        cb_translation_error = cv::Point3d(e_x, e_y, 0);
    }

    void fixed_parameters_t::load(const CustomNodeHandle &n)
    {
        ROS_ASSERT(n.getParam("import_path", import_path));
        ROS_ASSERT(n.NodeHandle::getParam("import_samples", import_samples));
        ROS_ASSERT(n.getParam("num_lowestvoq", num_lowestvoq));
        ROS_ASSERT(n.NodeHandle::getParam("queue_rate", queue_rate));
        ROS_ASSERT(n.NodeHandle::getParam("distance_offset_mm", distance_offset));
    }
}  // namespace cam_lidar_calibration

