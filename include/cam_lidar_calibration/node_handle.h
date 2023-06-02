#ifndef src_node_handle_h
#define src_node_handle_h

#include "filesystem.h"
#include <ros/param.h>
#include <ros/node_handle.h>

namespace cam_lidar_calibration
{
    struct CustomNodeHandle : public ros::NodeHandle
    {
        ~CustomNodeHandle() = default;

        CustomNodeHandle() = default;

        explicit CustomNodeHandle(const std::string &ns) : ros::NodeHandle(ns)
        {}

        bool getParam(const std::string &key, size_t &value) const
        {
            int i = 0;
            const bool ret = ros::param::get(resolveName(key), i);
            if (ret)
            {
                value = static_cast<size_t>(i);
            }
            return ret;
        }

        bool getParam(const std::string &key, fs::path &value) const
        {
            std::string s;
            const bool ret = ros::param::get(resolveName(key), s);
            if (ret)
            {
                value = fs::path(s);
            }
            return ret;
        }
    };

} // end namespace cam_lidar_calibration

#endif //src_node_handle_h
