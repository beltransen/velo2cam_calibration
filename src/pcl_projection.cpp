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
  pcl_projection: Project the cloud points to the image
*/

#define DEBUG 1

#include "velo2cam_utils.h"
#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace sensor_msgs;

image_transport::Publisher pub;

typedef struct {
  double r,g,b;
} COLOUR;

COLOUR // Color map
GetColour(double v,double vmin,double vmax)
{
  COLOUR c = {1.0,1.0,1.0}; // white
  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  } else {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }

  return(c);
}

void // Get the TF as an Eigen matrix
transformAsMatrix (const tf::Transform& bt, Eigen::Matrix4f &out_mat)
{
  double mv[12];
  bt.getBasis ().getOpenGLSubMatrix (mv);

  tf::Vector3 origin = bt.getOrigin ();

  out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

  out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  out_mat (0, 3) = origin.x ();
  out_mat (1, 3) = origin.y ();
  out_mat (2, 3) = origin.z ();
}

void // Get the camera_info P as an Eigen matrix
projectionAsMatrix (const CameraInfoConstPtr& cinfo_msg, Eigen::Matrix4f &out_mat)
{
  for (int i = 0, k = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++, k++){
      out_mat(i, j) = cinfo_msg->P[k];
    }
  }
  out_mat(3,0) = out_mat(3,1) = out_mat(3,2) = 0;
  out_mat(3,3) = 1;
}

void callback(const PointCloud2::ConstPtr& pcl_msg, const CameraInfoConstPtr& cinfo_msg, const ImageConstPtr& image_msg){


  pcl::PointCloud<Velodyne::Point>::Ptr pcl_cloud(new pcl::PointCloud<Velodyne::Point>); // From ROS Msg
  pcl::PointCloud<Velodyne::Point>::Ptr filtered_pcl_cloud(new pcl::PointCloud<Velodyne::Point>); // Only forward points
  pcl::PointCloud<Velodyne::Point>::Ptr trans_cloud(new pcl::PointCloud<Velodyne::Point>); // After transformation

  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Get the image
  cv::Mat image;
  try{
    image = cv_bridge::toCvShare(image_msg)->image;
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get the cloud
  fromROSMsg(*pcl_msg, *pcl_cloud);
  Velodyne::addRange(*pcl_cloud);

  // Get the TF
  if(DEBUG) ROS_INFO("Waiting for TF");
  try{
    listener.waitForTransform(image_msg->header.frame_id.c_str(), pcl_msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform (image_msg->header.frame_id.c_str(), pcl_msg->header.frame_id.c_str(), ros::Time(0), transform);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[pcl_projection] TF exception:\n%s", ex.what());
    return;
  }
  if(DEBUG) ROS_INFO("TF available");

  // Filter points behind image plane (approximation)
  pcl::PassThrough<Velodyne::Point> pass;
  pass.setInputCloud (pcl_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.0, 100.0);
  pass.filter (*filtered_pcl_cloud);

  // Move the lidar cloud to the camera frame
  Eigen::Matrix4f transf_mat;
  transformAsMatrix(transform, transf_mat);

  // Thanks @irecorte for this procedure:
  // Get the 4x4 projection matrix
  Eigen::Matrix4f projection_matrix;
  projectionAsMatrix(cinfo_msg, projection_matrix);

  // Get the array of lidar points
  Eigen::MatrixXf points_all = filtered_pcl_cloud->getMatrixXfMap();

  // Only x,y,z,1 (homogeneous coordinates)
  Eigen::MatrixXf points = points_all.topRows(4);
  points.row(3) =  Eigen::MatrixXf::Constant(1, points_all.cols(), 1);

  // Projection
  Eigen::MatrixXf points_img =  projection_matrix * transf_mat * points;
  points_img.row(0) = points_img.row(0).cwiseQuotient(points_img.row(2));
  points_img.row(1) = points_img.row(1).cwiseQuotient(points_img.row(2));

  // Draw the points
  for (int i = 0; i < points_img.cols(); i++){
    cv::Point2i p(points_img(0,i), points_img(1,i));
    // Only points within the image
    if (p.x >= 0 && p.x < cinfo_msg->width && p.y >= 0 && p.y < cinfo_msg->height){
      // Apply colormap to depth
      float depth = filtered_pcl_cloud->points[i].range;
      // Want it faster? Try depth = transf_points(2, i)
      COLOUR c = GetColour(int(depth/20.0*255.0), 0, 255);
      cv::circle(image, p, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
    }
  }

  // Publish
  pub.publish(cv_bridge::CvImage(image_msg->header, image_msg->encoding, image).toImageMsg());
  // if(DEBUG) ROS_INFO("Done");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pcl_projection");
  ros::NodeHandle nh_("~"); // LOCAL
  image_transport::ImageTransport it(nh_);

  // Subscribers
  message_filters::Subscriber<PointCloud2> pc_sub(nh_, "pointcloud", 1);
  message_filters::Subscriber<CameraInfo> cinfo_sub(nh_, "camera_info", 1);
  message_filters::Subscriber<Image> image_sub(nh_, "image", 1);

  pub = it.advertise("image_with_points", 1);

  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, CameraInfo, Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pc_sub, cinfo_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();
  return 0;
}
