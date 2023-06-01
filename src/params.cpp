#include "cam_lidar_calibration/params.h"

namespace cam_lidar_calibration
{
    void loadParams(const ros::NodeHandle& n, initial_parameters_t& i_params)
    {
        int cb_w, cb_h, w, h, e_x, e_y;
        ROS_ASSERT(n.getParam("camera_topic", i_params.camera_topic));
        ROS_ASSERT(n.getParam("camera_info", i_params.camera_info));
        ROS_ASSERT(n.getParam("lidar_topic", i_params.lidar_topic));
        ROS_ASSERT(n.getParam("chessboard/pattern_size/width", cb_w));
        ROS_ASSERT(n.getParam("chessboard/pattern_size/height", cb_h));
        i_params.chessboard_pattern_size = cv::Size(cb_w, cb_h);
        ROS_ASSERT(n.getParam("chessboard/square_length", i_params.square_length));
        ROS_ASSERT(n.getParam("chessboard/board_dimension/width", w));
        ROS_ASSERT(n.getParam("chessboard/board_dimension/height", h));
        i_params.board_dimensions = cv::Size(w, h);
        ROS_ASSERT(n.getParam("chessboard/translation_error/x", e_x));
        ROS_ASSERT(n.getParam("chessboard/translation_error/y", e_y));
        i_params.cb_translation_error = cv::Point3d(e_x, e_y, 0);
    }

    void loadParams(const CustomNodeHandle &n, fixed_parameters_t &f_params)
    {
        ROS_ASSERT(n.getParam("import_path", f_params.import_path));
        ROS_ASSERT(n.NodeHandle::getParam("import_samples", f_params.import_samples));
        ROS_ASSERT(n.getParam("num_lowestvoq", f_params.num_lowestvoq));
        ROS_ASSERT(n.NodeHandle::getParam("queue_rate", f_params.queue_rate));
        ROS_ASSERT(n.NodeHandle::getParam("distance_offset_mm", f_params.distance_offset));
    }
}  // namespace cam_lidar_calibration

