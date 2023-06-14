#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>

#include "cam_lidar_calibration/params.h"
#include "cam_lidar_calibration/Optimise.h"
#include "cam_lidar_calibration/optimiser.h"
#include "cam_lidar_calibration/point_xyzir.h"
#include "cam_lidar_calibration/boundsConfig.h"
#include "cam_lidar_calibration/RunOptimiseAction.h"


using image_sub_type = message_filters::Subscriber<sensor_msgs::Image>;
using pc_sub_type = message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZIR>>;
using ImageLidarSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZIR>>;

namespace cam_lidar_calibration
{
    geometry_msgs::Quaternion normalToQuaternion(const cv::Point3d &normal);

    class FeatureExtractor
    {
    public:
        FeatureExtractor();

        ~FeatureExtractor() = default;

        void extractRegionOfInterest();

        void syncCallback(const sensor_msgs::Image::ConstPtr &img,
                          const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &pc);

        bool captureCallback(Optimise::Request &req, Optimise::Response &res);

        void reconfCallback(const cam_lidar_calibration::boundsConfig &config, uint32_t level);

        void optimise(const RunOptimiseGoalConstPtr& goal,
                      actionlib::SimpleActionServer<RunOptimiseAction>* as);

        void visualiseSamples() const;

        void visualiseBoundedCloud();

        bool importSamples() const;

        bool initialized() const;

    private:
        std::tuple<std::vector<cv::Point3d>, cv::Mat>
        extractChessboard(const sensor_msgs::Image::ConstPtr &image);
        
        std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>
        extractBoard3D(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &cloud) const;
        
        std::pair<pcl::ModelCoefficients, pcl::ModelCoefficients>
        extractEdges(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &edge_pair_cloud) const;
        
        std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point3d>>
        chessboardProjection(const std::vector<cv::Point2d> &corners);
        
        void distoffsetPassthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &input_pc,
                                   pcl::PointCloud<pcl::PointXYZIR>::Ptr &output_pc) const;
        
        void passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &input_pc,
                         pcl::PointCloud<pcl::PointXYZIR>::Ptr &output_pc) const;

        void setCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &msg);

        bool getLastCloud(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &out);

        bool getLastData(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &pc_out, sensor_msgs::Image::ConstPtr &img_out);

        void setLastData(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &pc_in,
                         const sensor_msgs::Image::ConstPtr &img_in);

        std::string getDateTime() const;

        int findOctant(float x, float y, float z) const;
        
        void publishBoardPointCloud() const;
        
        void publishChessboard() const;
    private:
        double meter_per_pixel_cbdiag_ {0};
        bool valid_camera_info_ {false};
        CustomNodeHandle private_nh_ {"~"};
        ros::NodeHandle public_nh_ {};
        cv::Mat last_normal_ {};
        cv::Mat chessboard_sample_ {};
        
        initial_parameters_t i_params_;
        fixed_parameters_t f_params_;
        
        sensor_msgs::Image::ConstPtr last_img_;
        pcl::PointCloud<pcl::PointXYZIR>::ConstPtr last_pcl_;
        
        std::shared_ptr<Optimiser> optimiser_;
        std::shared_ptr<std::mutex> last_data_mutex_;
        std::shared_ptr<std::atomic<int>> flag_;
        std::shared_ptr<image_sub_type> image_sub_;
        std::shared_ptr<pc_sub_type> pc_sub_;
        std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> image_pc_sync_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        std::shared_ptr<image_transport::ImageTransport> it_p_;
        std::shared_ptr<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>> server;
        
        ros::Publisher board_cloud_pub_;
        ros::Publisher bounded_cloud_pub_;
        ros::Publisher samples_pub_;
        image_transport::Publisher image_publisher_;
        ros::ServiceServer optimise_service_;
        cam_lidar_calibration::boundsConfig bounds_;
        
        std::vector<cv::Point2f> centresquare_corner_pixels_;
        std::vector<pcl::PointCloud<pcl::PointXYZIR>::Ptr> pc_samples_;
        
        std::string lidar_frame_;
        std::string curdatetime_;
        
        fs::path pkg_path_;
        fs::path data_path_;
    };

}  // namespace cam_lidar_calibration
