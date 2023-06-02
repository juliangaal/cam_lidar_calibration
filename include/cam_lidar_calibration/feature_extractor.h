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

        bool serviceCB(Optimise::Request &req, Optimise::Response &res);

        void boundsCB(boundsConfig &config, uint32_t level);

        void optimise(const RunOptimiseGoalConstPtr &goal,
                      actionlib::SimpleActionServer<cam_lidar_calibration::RunOptimiseAction> *as);

        void visualiseSamples();

        void visualiseBoundedCloud();

        bool importSamples() const;

        bool initialized() const;

    private:
        void passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &input_pc,
                         pcl::PointCloud<pcl::PointXYZIR>::Ptr &output_pc);

        std::tuple<std::vector<cv::Point3d>, cv::Mat> extractChessboard(const cv_bridge::CvImageConstPtr &cv_pt);

        auto chessboardProjection(const std::vector<cv::Point2d> &corners,
                                  const cv_bridge::CvImageConstPtr &cv_ptr);

        void publishBoardPointCloud();

        std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>
        extractBoard(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &cloud, OptimisationSample &sample);

        std::pair<pcl::ModelCoefficients, pcl::ModelCoefficients>
        findEdges(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &edge_pair_cloud);

        void setCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &msg);

        void distoffsetPassthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &input_pc,
                              pcl::PointCloud<pcl::PointXYZIR>::Ptr &output_pc);

        bool getLastCloud(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &out);

        bool getLastData(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &pc_out, cv_bridge::CvImageConstPtr &img_out);

        void setLastData(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr &pc_in,
                         const sensor_msgs::Image::ConstPtr &img_in);

        std::string getDateTime();

        int findOctant(float x, float y, float z);

        std::shared_ptr<Optimiser> optimiser_;
        initial_parameters_t i_params_;
        fixed_parameters_t f_params_;
        std::vector<cv::Point2f> centresquare_corner_pixels_;
        double metreperpixel_cbdiag_;
        std::string lidar_frame_;
        std::shared_ptr<std::mutex> last_data_mutex_;
        pcl::PointCloud<pcl::PointXYZIR>::ConstPtr last_pcl_;
        cv_bridge::CvImageConstPtr last_img_;
        std::shared_ptr<std::atomic<int>> flag_;
        cam_lidar_calibration::boundsConfig bounds_;
        std::shared_ptr<image_sub_type> image_sub_;
        std::shared_ptr<pc_sub_type> pc_sub_;
        std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> image_pc_sync_;
        int num_samples_;
        bool valid_camera_info_;
        std::vector<pcl::PointCloud<pcl::PointXYZIR>::Ptr> pc_samples_;
        ros::Publisher board_cloud_pub_, bounded_cloud_pub_;
        ros::Publisher samples_pub_;
        image_transport::Publisher image_publisher_;
        ros::ServiceServer optimise_service_;
        boost::shared_ptr<image_transport::ImageTransport> it_;
        boost::shared_ptr<image_transport::ImageTransport> it_p_;
        boost::shared_ptr<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>> server;
        std::string curdatetime_;
        fs::path output_path_;
        fs::path newdatafolder_;
        CustomNodeHandle private_nh_;
        ros::NodeHandle public_nh_;
    };

}  // namespace cam_lidar_calibration
