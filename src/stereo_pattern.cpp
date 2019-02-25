/*
  velo2cam_calibration - Automatic calibration algorithm for extrinsic parameters of a stereo camera and a velodyne
  Copyright (C) 2017-2018 Jorge Beltran, Carlos Guindel

  This file is part of velo2cam_calibration.

  velo2cam_calibration is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  velo2cam_calibration is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with velo2cam_calibration.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  stereo_pattern: Find the circle centers in the stereo cloud
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <dynamic_reconfigure/server.h>

#include <velo2cam_calibration/CameraConfig.h>
#include "velo2cam_utils.h"
#include <velo2cam_calibration/ClusterCentroids.h>

using namespace std;
using namespace sensor_msgs;

int nFrames;
int images_proc_=0, images_used_=0;

double circle_threshold_;
double line_threshold_;
double min_plane_normal_z_;
double plane_distance_inliers_;
double min_border_x_;
double min_distance_between_borders_x_;
double min_distance_between_borders_y_;
double border_distance_inliers_;
int min_line_inliers_;
double cluster_size_;
int min_centers_found_;

ros::Publisher inliers_pub;
ros::Publisher coeff_pub;
ros::Publisher boundary_edges_pub;
ros::Publisher occluding_edges_pub;
ros::Publisher occluded_edges_pub;
ros::Publisher high_curvature_edges_pub;
ros::Publisher rgb_edges_pub;
ros::Publisher plane_edges_pub;
ros::Publisher circles_pub;
ros::Publisher xy_pattern_pub;
ros::Publisher no_circles_pub;
ros::Publisher cumulative_pub;
ros::Publisher final_pub;
ros::Publisher transf_pub;
ros::Publisher auxpoint_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;

std_msgs::Header header_;

void publish_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, ros::Publisher pub){
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*input_cloud, ros_cloud);
  ros_cloud.header = header_;
  pub.publish(ros_cloud);
}

void callback(const PointCloud2::ConstPtr& camera_cloud,
              const pcl_msgs::ModelCoefficients::ConstPtr& cam_plane_coeffs){

  if(DEBUG) ROS_INFO("[Camera] Processing image...");

  images_proc_++;

  header_ = camera_cloud->header;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::fromROSMsg(*camera_cloud,*cam_cloud);

  // 1.FILTER THE ONLY-EDGES-CLOUD ACCORDING TO THE DETECTED PLANE
  // Make sure that the plane makes sense
  if (cam_plane_coeffs->values[2]<min_plane_normal_z_){
    ROS_WARN("[Camera] Estimated plane is not valid.");
    return;
  }

  // Segment planecircles_pub2
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = cam_plane_coeffs->values[0];
  coefficients_v(1) = cam_plane_coeffs->values[1];
  coefficients_v(2) = cam_plane_coeffs->values[2];
  coefficients_v(3) = cam_plane_coeffs->values[3];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_plane_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr dit (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cam_cloud));
  std::vector<int> plane_inliers;
  dit->selectWithinDistance (coefficients_v, plane_distance_inliers_, plane_inliers);
  pcl::copyPointCloud<pcl::PointXYZ>(*cam_cloud, plane_inliers, *cam_plane_cloud);

  // Publish plane as "plane_edges_cloud"
  PointCloud2 plane_edges_ros;
  pcl::toROSMsg(*cam_plane_cloud, plane_edges_ros);
  plane_edges_ros.header = camera_cloud->header;
  plane_edges_pub.publish(plane_edges_ros);

  // 2.REMOVE BORDER LINES
  // Find them
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> lineseg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::vector<pcl::ModelCoefficients> out_lines;

  lineseg.setModelType (pcl::SACMODEL_LINE);
  lineseg.setDistanceThreshold (line_threshold_);
  lineseg.setMethodType (pcl::SAC_RANSAC);
  lineseg.setOptimizeCoefficients (true);
  lineseg.setMaxIterations(1000);

  pcl::copyPointCloud<pcl::PointXYZ>(*cam_plane_cloud, *temp_line_cloud);

  float first_line_x = -1.0, first_line_y = -1.0;
  int last_inliers;

  do{
    lineseg.setInputCloud (temp_line_cloud);
    lineseg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      if (DEBUG) ROS_INFO("Could not estimate a line model for the given dataset.");
      break;
    }

    last_inliers = inliers->indices.size ();

    if (inliers->indices.size () < min_line_inliers_){
      continue;
    }

    // Extract the inliers
    extract.setInputCloud (temp_line_cloud);
    extract.setIndices (inliers);

    // Remove inliers from cloud to find next line
    extract.setNegative (true);
    extract.filter(*temp_cloud);
    temp_line_cloud.swap(temp_cloud);

    //if(DEBUG) ROS_INFO("Line orientation %f %f %f", coefficients->values[3], coefficients->values[4], coefficients->values[5 ]);

    if (fabs(coefficients->values[3])<min_border_x_ && fabs(coefficients->values[4])<min_border_x_){
      //if(DEBUG) ROS_INFO("Invalid line (orientation)");
      continue;
    }

    if (fabs(coefficients->values[3]) >= min_border_x_){
      if (first_line_x<0.0) first_line_x = coefficients->values[0];
      else if (fabs(coefficients->values[0]-first_line_x) < min_distance_between_borders_x_){
        //if(DEBUG) ROS_INFO("Invalid line (proximity)");
        continue;
      }
    }else if(fabs(coefficients->values[4]) >= min_border_x_){
      if (first_line_y<0.0) first_line_y = coefficients->values[1];
      else if (fabs(coefficients->values[1]-first_line_y) < min_distance_between_borders_y_){
        //if(DEBUG) ROS_INFO("Invalid line (proximity)");
        continue;
      }
    }

    if(DEBUG) ROS_INFO("[Camera] Removed line: %d inliers / orientation: %f %f %f",
    last_inliers, coefficients->values[3], coefficients->values[4],
    coefficients->values[5]);

    out_lines.push_back(*coefficients);

  }while (last_inliers > min_line_inliers_ && out_lines.size() < 4);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_plane_wol_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // Remove them
  for (vector<pcl::ModelCoefficients>::iterator it=out_lines.begin(); it<out_lines.end(); ++it){

    pcl::PointIndices::Ptr line_ind (new pcl::PointIndices);
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr dil (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cam_plane_cloud));
    Eigen::VectorXf coefficients_l(6);
    for(int j=0; j<6; j++){
      coefficients_l(j) = it->values[j];
    }
    std::vector<int> line_inliers;
    dil->selectWithinDistance (coefficients_l, border_distance_inliers_, line_inliers);

    line_ind->indices.resize(line_inliers.size());

    std::copy(line_inliers.begin(), line_inliers.end(), line_ind->indices.begin());

    // Extract the inliers
    extract.setInputCloud (cam_plane_cloud);
    extract.setIndices (line_ind);
    extract.setNegative (true);
    extract.filter (*cam_plane_wol_cloud);

    cam_plane_wol_cloud.swap(cam_plane_cloud);
  }

  PointCloud2 circles_ros;
  pcl::toROSMsg(*cam_plane_cloud, circles_ros);
  circles_ros.header = camera_cloud->header;
  circles_pub.publish(circles_ros);

  // 3.ROTATE CLOUD TO FACE PATTERN PLANE
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3d xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;

  floor_plane_normal_vector[0] = cam_plane_coeffs->values[0];
  floor_plane_normal_vector[1] = cam_plane_coeffs->values[1];
  floor_plane_normal_vector[2] = cam_plane_coeffs->values[2];

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*cam_plane_cloud, *cam_plane_cloud, indices);

  Eigen::Affine3d rotation = getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*cam_plane_cloud, *xy_cloud, rotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3)/coefficients_v(2));
  aux_cloud->push_back(aux_point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr auxrotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);

  sensor_msgs::PointCloud2 ros_auxpoint;
  pcl::toROSMsg(*aux_cloud, ros_auxpoint);
  ros_auxpoint.header = camera_cloud->header;
  auxpoint_pub.publish(ros_auxpoint);

  double zcoord_xyplane = auxrotated_cloud->at(0).z;

  // Force pattern points to belong to computed plane
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = xy_cloud->points.begin();
  pt < xy_cloud->points.end(); ++pt){
    pt->z = zcoord_xyplane;
  }

  PointCloud2 xy_pattern_ros;
  pcl::toROSMsg(*xy_cloud, xy_pattern_ros);
  xy_pattern_ros.header = camera_cloud->header;
  xy_pattern_pub.publish(xy_pattern_ros);

  // 4.FIND CIRCLES
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  seg.setDistanceThreshold (circle_threshold_);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients (true);
  seg.setMaxIterations(1000);
  seg.setRadiusLimits(0.11,0.13);

  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector< std::vector<float> > found_centers;
  bool valid = true;

  do{

    seg.setInputCloud (xy_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_WARN ("[Camera] Could not estimate a circle model for the given dataset.");
      break;
    }else{
      if(DEBUG) ROS_INFO ("[Camera] Found circle: %lu inliers", inliers->indices.size ());
    }

    // Extract the inliers
    extract.setInputCloud (xy_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*circle_cloud);

    // Add center point to cloud
    pcl::PointXYZ center;
    center.x = *coefficients->values.begin();
    center.y = *(coefficients->values.begin()+1);
    for(std::vector<std::vector <float> >::iterator it = found_centers.begin(); it != found_centers.end(); ++it) {
      if (sqrt(pow(fabs((*it)[0]-center.x),2) + pow(fabs((*it)[1]-center.y),2))<0.25){
        valid = false;
      }
    }

    center.z = zcoord_xyplane;

    if (valid){
      std::vector<float> found_center;
      found_center.push_back(center.x);
      found_center.push_back(center.y);
      found_center.push_back(center.z);
      found_centers.push_back(found_center);
    }

    // Remove inliers from pattern cloud to find next circle
    extract.setNegative (true);
    extract.filter(*temp_cloud);
    xy_cloud.swap(temp_cloud);

    PointCloud2 no_circles_ros;
    pcl::toROSMsg(*xy_cloud, no_circles_ros);
    no_circles_ros.header = camera_cloud->header;
    no_circles_pub.publish(no_circles_ros);

  }while(found_centers.size() < 4 && valid);

  if (found_centers.size() < min_centers_found_ || found_centers.size() > 4){
    ROS_WARN("Not enough centers: %ld", found_centers.size());
    return;
  }else{
    for (std::vector<std::vector<float> >::iterator it = found_centers.begin(); it < found_centers.end(); ++it){
      pcl::PointXYZ center;
      center.x = (*it)[0];
      center.y = (*it)[1];
      center.z = (*it)[2];;
      pcl::PointXYZ center_rotated_back = pcl::transformPoint(center, rotation.inverse());
      center_rotated_back.z = (-cam_plane_coeffs->values[0]*center_rotated_back.x - cam_plane_coeffs->values[1]*center_rotated_back.y - cam_plane_coeffs->values[3])/cam_plane_coeffs->values[2];
      cumulative_cloud->push_back(center_rotated_back);
    }
  }

  PointCloud2 cum_ros;
  pcl::toROSMsg(*cumulative_cloud, cum_ros);
  cum_ros.header = camera_cloud->header;
  cumulative_pub.publish(cum_ros);

  pcl_msgs::PointIndices p_ind;

  pcl_conversions::moveFromPCL(*inliers, p_ind);
  p_ind.header = camera_cloud->header;

  pcl_msgs::ModelCoefficients m_coeff;

  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = camera_cloud->header;

  inliers_pub.publish(p_ind);
  coeff_pub.publish(m_coeff);

  nFrames++;
  images_used_ = nFrames;
  ROS_INFO("[Camera] %d/%d frames: %ld pts in cloud",
  images_used_, images_proc_, cumulative_cloud->points.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  getCenterClusters(cumulative_cloud, final_cloud, 0.1, nFrames/2, nFrames);
  if (final_cloud->points.size()>4){
    getCenterClusters(cumulative_cloud, final_cloud, 0.1, 3.0*nFrames/4.0, nFrames);
  }

  if (final_cloud->points.size()==4){

    sensor_msgs::PointCloud2 final_ros;
    pcl::toROSMsg(*final_cloud, final_ros);
    final_ros.header = camera_cloud->header;

    velo2cam_calibration::ClusterCentroids to_send;
    to_send.header = camera_cloud->header;
    to_send.total_iterations = images_proc_;
    to_send.cluster_iterations = images_used_;
    to_send.cloud = final_ros;

    final_pub.publish(to_send);
  }
}

void param_callback(velo2cam_calibration::CameraConfig &config, uint32_t level){
  circle_threshold_ = config.circle_threshold;
  ROS_INFO("New circle threshold: %f", circle_threshold_);
  line_threshold_ = config.line_threshold;
  ROS_INFO("New line threshold: %f", line_threshold_);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "stereo_pattern");
  ros::NodeHandle nh_("~");

  message_filters::Subscriber<PointCloud2> camera_cloud_sub_;
  message_filters::Subscriber<pcl_msgs::ModelCoefficients> cam_plane_coeffs_sub_;

  camera_cloud_sub_.subscribe(nh_, "cloud2", 1);
  cam_plane_coeffs_sub_.subscribe(nh_, "cam_plane_coeffs", 1);

  inliers_pub = nh_.advertise<pcl_msgs::PointIndices> ("inliers", 1);
  coeff_pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("model", 1);
  plane_edges_pub = nh_.advertise<PointCloud2> ("plane_edges_cloud", 1);
  circles_pub = nh_.advertise<PointCloud2> ("circles_cloud", 1);
  xy_pattern_pub = nh_.advertise<PointCloud2> ("xy_pattern", 1);
  no_circles_pub = nh_.advertise<PointCloud2> ("no_circles", 1);
  cumulative_pub = nh_.advertise<PointCloud2> ("cumulative_cloud", 1);
  final_pub = nh_.advertise<velo2cam_calibration::ClusterCentroids> ("centers_cloud", 1);
  transf_pub = nh_.advertise<PointCloud2> ("transf_cam", 1);
  auxpoint_pub= nh_.advertise<PointCloud2> ("aux_point", 1);

  cumulative_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  typedef message_filters::sync_policies::ExactTime<PointCloud2, pcl_msgs::ModelCoefficients> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10), camera_cloud_sub_, cam_plane_coeffs_sub_);
  sync_.registerCallback(boost::bind(&callback, _1, _2));

  nh_.param("min_plane_normal_z", min_plane_normal_z_, 0.96);
  nh_.param("plane_distance_inliers", plane_distance_inliers_, 0.01);
  nh_.param("min_border_x", min_border_x_, 0.96);
  nh_.param("min_distance_between_borders_x", min_distance_between_borders_x_, 1.0);
  nh_.param("min_distance_between_borders_y", min_distance_between_borders_y_, 0.6);
  nh_.param("border_distance_inliers", border_distance_inliers_, 0.05);
  nh_.param("min_line_inliers", min_line_inliers_, 1200); //TODO: Adapt to the distance to the plane
  nh_.param("cluster_size", cluster_size_, 0.02);
  nh_.param("min_centers_found", min_centers_found_, 4);

  dynamic_reconfigure::Server<velo2cam_calibration::CameraConfig> server;
  dynamic_reconfigure::Server<velo2cam_calibration::CameraConfig>::CallbackType f;
  f = boost::bind(param_callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
