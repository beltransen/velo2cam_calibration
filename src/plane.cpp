/******************************************************************************
* plane_segmentation node
* Extracts the ground plane from a pointcloud
* Inputs: input (sensor_msgs::PointCloud2): input pointcloud
* Ouputs: inliers (pcl_msgs::PointIndices): ground plane inliers
*         model (pcl_msgs::ModelCoefficients): plane coefficients
* Parameters: Uses dynamic_reconfigure to set the threshold
* Mantainer: cguindel@ing.uc3m.es
******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <dynamic_reconfigure/server.h>
#include <velo2cam_calibration/PlaneConfig.h>

class SACSegmentator
{
public:

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    ros::Publisher inliers_pub;
    ros::Publisher coeff_pub;
    double threshold_;
    int sac_segmentation_type_;
    double eps_angle_;

    Eigen::Vector3f Axis;

    dynamic_reconfigure::Server<velo2cam_calibration::PlaneConfig> server;
    dynamic_reconfigure::Server<velo2cam_calibration::PlaneConfig>::CallbackType f;

    SACSegmentator():
        nh_("~"), threshold_(0.1)
    {
        cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("input", 1, &SACSegmentator::cloud_callback, this);
        inliers_pub = nh_.advertise<pcl_msgs::PointIndices> ("inliers", 1);
        coeff_pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("model", 1);

        std::vector<double> axis_param;
        Axis = Eigen::Vector3f::Zero();

        nh_.getParam("axis", axis_param);

        if (axis_param.size()==3){
            for (int i=0; i<3; ++i){
                Axis[i] = axis_param[i];
            }
        }else{
            Axis[0]=0.0; Axis[1]=1.0; Axis[2]=0.0;
        }

        nh_.param("eps_angle", eps_angle_, 0.35);

        // 0: SACMODEL_NORMAL_PLANE
        // 1: SACMODEL_PARALLEL_PLANE
        nh_.param("segmentation_type", sac_segmentation_type_, 0);

        if (sac_segmentation_type_ == 0){
          ROS_INFO("Searching for planes perpendicular to %f %f %f", Axis[0], Axis[1], Axis[2]);
        }else{
          ROS_INFO("Searching for planes parallel to %f %f %f", Axis[0], Axis[1], Axis[2]);
        }

        f = boost::bind(&SACSegmentator::param_callback, this, _1, _2);
        server.setCallback(f);

        ROS_INFO("Segmentator ready");

    }

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromROSMsg(*input,*cloud);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        seg.setModelType (sac_segmentation_type_ ? pcl::SACMODEL_PARALLEL_PLANE : pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setDistanceThreshold (threshold_);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setAxis(Axis);
        seg.setOptimizeCoefficients (true);

        seg.setEpsAngle(eps_angle_);
        seg.setMaxIterations(250);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud (*cloud, *cloud, indices);

        seg.setInputCloud (cloud->makeShared());
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
          ROS_WARN ("Could not estimate a planar model for the given dataset.");
          return;
        }

        pcl_msgs::PointIndices p_ind;

        pcl_conversions::moveFromPCL(*inliers, p_ind);
        p_ind.header = input->header;

        pcl_msgs::ModelCoefficients m_coeff;

        pcl_conversions::moveFromPCL(*coefficients, m_coeff);
        m_coeff.header = input->header;

        inliers_pub.publish(p_ind);
        coeff_pub.publish(m_coeff);
    }

    void param_callback(velo2cam_calibration::PlaneConfig &config, uint32_t level){
        threshold_ = config.threshold;
        ROS_INFO("New distance threshold: %f", threshold_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plane_segmentation");

    SACSegmentator seg;

    ros::spin();

}
