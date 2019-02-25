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
  Includes code from https://github.com/Flos/ros-image_cloud licensed
  under the MIT license, set out below.

  The MIT License (MIT)

  Copyright (c) 2015

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/*
    Calculates miscalibration value according to:
    J. Levinson and S. Thrun, “Automatic Online Calibration of Cameras and Lasers,”
    in Robotics: Science and Systems, 2013.
*/

#define PCL_NO_PRECOMPILE

#include "velo2cam_utils.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <image_geometry/pinhole_camera_model.h>

bool laserReceived, cameraReceived;
pcl::PointCloud<Velodyne::Point>::Ptr edges_cloud;
ros::Publisher cloud_pub;
cv::Mat img_out;
std::string target_frame, source_frame;
bool listen_to_tf_, save_to_file_;
int step_;
std::ofstream savefile;

typedef struct {
  double r,g,b;
} COLOUR;

COLOUR GetColour(double v,double vmin,double vmax)
{
  COLOUR c = {1.0,1.0,1.0}; // white
  double dv;

  if (v < vmin)
  v = vmin;
  if (v > vmax)
  v = vmax;
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

void edges_pointcloud(pcl::PointCloud<Velodyne::Point>::Ptr pc){
  std::vector<std::vector<Velodyne::Point*> > rings = Velodyne::getRings(*pc);
  for (std::vector<std::vector<Velodyne::Point*> >::iterator ring = rings.begin(); ring < rings.end(); ++ring){
    if (ring->empty()) continue;

    (*ring->begin())->intensity = 0;
    (*(ring->end() - 1))->intensity = 0;
    for (vector<Velodyne::Point*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; ++pt){
      Velodyne::Point *prev = *(pt - 1);
      Velodyne::Point *succ = *(pt + 1);
      (*pt)->intensity = pow(std::max( std::max( prev->range-(*pt)->range, succ->range-(*pt)->range), 0.f), 0.5);
    }
  }
}

template <class T>
inline bool between(T min, T test, T max){
  if( min < test && test < max ) return true;
  return false;
}

template <typename ImageT>
inline
void get_diff(int col, int row, int col_c, int row_c, int &result, cv::Mat& in) {
  if (between(-1, col, in.cols) && between(-1, row, in.rows)) {
    // hit upper left
    result = std::max(abs(in.at<ImageT>(row, col) - in.at<ImageT>(row_c, col_c)),	result);
  }
}

template <typename ImageT>
inline
int max_diff_neighbors(int row_c, int col_c, cv::Mat &in){
  //Check
  int result = 0;

  get_diff<ImageT>(col_c -1, row_c -1, col_c, row_c, result, in);
  get_diff<ImageT>(col_c	  , row_c -1, col_c, row_c, result, in);
  get_diff<ImageT>(col_c +1, row_c -1, col_c, row_c, result, in);
  get_diff<ImageT>(col_c -1, row_c   , col_c, row_c, result, in);
  get_diff<ImageT>(col_c +1, row_c   , col_c, row_c, result, in);
  get_diff<ImageT>(col_c -1, row_c +1, col_c, row_c, result, in);
  get_diff<ImageT>(col_c   , row_c +1, col_c, row_c, result, in);
  get_diff<ImageT>(col_c +1, row_c +1, col_c, row_c, result, in);

  return result;
}

template <typename ImageT>
inline void
edge_max(cv::Mat &in, cv::Mat &out)
{
  assert(in.rows == out.rows);
  assert(in.cols == out.cols);
  assert(in.depth() == out.depth());
  assert(in.channels() == out.channels());

  for(int r = 0; r < in.rows; r++)
  {
    for(int c = 0; c < in.cols; c++)
    {
      out.at<ImageT>(r,c) = max_diff_neighbors<ImageT>(r, c, in);
    }
  }
}

template <typename Type_in, typename Type_out>
inline float calc(float &val, const float &psi, int row, int col, const cv::Mat& in, cv::Mat& out) {

  val = val * psi; /* Fade out the value */

  if (in.at<Type_in>(row, col) > val) /* In Value in the image bigger than the current value */
  {
    val = in.at<Type_in>(row, col); /* yes, get the bigger value */
  }

  if (out.at<Type_out>(row, col) < val) /* is the calculated value bigger then the value in the filtered image? */
  {
    out.at<Type_out>(row, col) = val; /* yes, store the calculated value in the filtered image */
  }
  else{
    val = out.at<Type_out>(row, col); /* no, the value of the filtered image is bigger use it */
  }

  return val;
}

template <typename Type_in, typename Type_out>
inline void
neighbors_x_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
  for(int row = 0; row < in.rows; ++row)
  {
    float val = 0;
    val = 0;
    for(int col = 0; col < in.cols; ++col)
    {
      val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
    }
  }
}

template <typename Type_in, typename Type_out>
inline void
neighbors_x_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
  float val = 0;

  for(int row = 0; row < in.rows; ++row)
  {
    val = 0;
    for(int col = in.cols -1; col >= 0; --col)
    {
      val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
    }
  }
}

template <typename Type_in, typename Type_out>
inline void
neighbors_y_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
  float val = 0;

  for(int col = 0; col < in.cols; ++col)
  {
    val = 0;
    for(int row = 0; row < in.rows; ++row)
    {
      val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
    }
  }
}

template <typename Type_in, typename Type_out>
inline void
neighbors_y_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
  float val = 0;

  for(int col = 0;  col < in.cols; ++col)
  {
    val = 0;
    for(int row = in.rows -1; row >= 0; --row)
    {
      val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
    }
  }
}

template <typename Type_in, typename Type_out>
void
inverse_distance_transformation(cv::Mat &in, cv::Mat &out, float alpha = 0.333333333, float psi = 0.98)
{
  assert(in.channels() == 1);
  assert(in.depth() == CV_8U);

  assert(in.size == out.size);
  assert(in.rows == out.rows);
  assert(in.cols == out.cols);

  neighbors_x_pos<Type_in, Type_out>(in, out, psi, alpha);
  neighbors_x_neg<Type_in, Type_out>(in, out, psi, alpha);
  neighbors_y_pos<Type_in, Type_out>(in, out, psi, alpha);
  neighbors_y_neg<Type_in, Type_out>(in, out, psi, alpha);

  for(int row = 0; row < in.rows; row++)
  {
    for(int col = 0; col < in.cols; col++)
    {
      int val = alpha * in.at<Type_in>(row,col) + (1 - alpha)*(float)out.at<Type_out>(row,col);
      out.at<Type_out>(row, col) = val;
    }
  }
}

void
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

void checkExtrinics(const sensor_msgs::CameraInfoConstPtr& cinfo_msg){

  if(!laserReceived || !cameraReceived){
    return;
  }

  ROS_INFO("Both received");

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cinfo_msg);

  pcl::PointCloud<Velodyne::Point>::Ptr trans_cloud(new pcl::PointCloud<Velodyne::Point>);

  double x,y,z;
  double r,p,yaw;

  tf::StampedTransform transform;
  if (listen_to_tf_){
    ROS_INFO("Using available tf transformation");
    tf::TransformListener listener;

    try{
      listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0), ros::Duration(20.0));
      listener.lookupTransform (target_frame.c_str(), source_frame.c_str(), ros::Time(0), transform);
    }catch (tf::TransformException& ex) {
      ROS_WARN("TF exception:\n%s", ex.what());
      return;
    }
    Eigen::Matrix4f transf_mat;
    transformAsMatrix(transform, transf_mat);
    pcl::transformPointCloud(*edges_cloud, *trans_cloud, transf_mat);

    trans_cloud->header.frame_id = target_frame;

    cv::Mat img_color;
    cv::cvtColor(img_out, img_color, cv::COLOR_GRAY2BGR);

    double objective_function = 0;

    ROS_INFO("Accumulating");

    for (pcl::PointCloud<Velodyne::Point>::iterator pt = trans_cloud->points.begin(); pt < trans_cloud->points.end(); pt++)
    {
      cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      COLOUR c = GetColour(int((*pt).intensity*255.0), 0, 255);

      cv::circle(img_color, uv, 4, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);

      if (uv.x>=0 && uv.x < img_out.cols && uv.y >= 0 && uv.y < img_out.rows){
        objective_function += img_out.at<uchar>(uv.y, uv.x)*(*pt).intensity;
      }

    }

    if (save_to_file_){
      savefile << transform.getOrigin()[0] << ", " << transform.getOrigin()[1] << ", " << transform.getOrigin()[2] << ", " << transform.getRotation()[0] << ", " << transform.getRotation()[1] << ", " << transform.getRotation()[2] << ", " << transform.getRotation()[3] << ", " << objective_function << ", " << endl;
    }

    ROS_INFO("Objective funcion: %lf", objective_function);

    cv::imshow("Final Levinson", img_color);
    cv::waitKey(3);
    edges_cloud.reset();

    laserReceived =false;
    cameraReceived = false;

    step_++;
    return;

  }else{

    tf::TransformListener listener;

    try{
      listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0), ros::Duration(20.0));
      listener.lookupTransform (target_frame.c_str(), source_frame.c_str(), ros::Time(0), transform);
    }catch (tf::TransformException& ex) {
      ROS_WARN("TF exception:\n%s", ex.what());
      return;
    }

    x= transform.getOrigin().getX();
    y= transform.getOrigin().getY();
    z= transform.getOrigin().getZ();
    transform.getBasis().getRPY(r,p,yaw);

  }

  ROS_INFO("%lf %lf %lf %lf %lf %lf", x, y, z, r, p, yaw);

  double x_mod, y_mod, z_mod;
  double r_mod, yaw_mod, p_mod;

  int MAX = 5;
  int M = (MAX-1)/2;
  double INC = 0.001, INC_ANG = 0.001;

  //#pragma omp parallel for
  for (int iter_x=0; iter_x<MAX; iter_x++){
    ROS_INFO("iter_x %d", iter_x);
    for (int iter_y=0; iter_y<MAX; iter_y++){
      for (int iter_z=0; iter_z<MAX; iter_z++){
        for (int iter_r=0; iter_r<MAX; iter_r++){
          for (int iter_p=0; iter_p<MAX; iter_p++){
            for (int iter_yaw=0; iter_yaw<MAX; iter_yaw++){
              x_mod = x + (iter_x-M)*INC;
              y_mod = y + (iter_y-M)*INC;
              z_mod = z + (iter_z-M)*INC;
              r_mod = r + (iter_r-M)*INC_ANG;
              p_mod = p + (iter_p-M)*INC_ANG;
              yaw_mod = yaw + (iter_yaw-M)*INC_ANG;

              tf::Vector3 origin(x_mod, y_mod, z_mod);
              tf::Quaternion q;
              q.setRPY(r_mod,p_mod,yaw_mod);
              transform.setOrigin(origin);
              transform.setRotation(q);

              Eigen::Matrix4f transf_mat;
              transformAsMatrix(transform, transf_mat);
              pcl::transformPointCloud(*edges_cloud, *trans_cloud, transf_mat);

              trans_cloud->header.frame_id = target_frame;

              cv::Mat img_color;
              cv::cvtColor(img_out, img_color, cv::COLOR_GRAY2BGR);

              double objective_function = 0;

              for (pcl::PointCloud<Velodyne::Point>::iterator pt = trans_cloud->points.begin(); pt < trans_cloud->points.end(); ++pt)
              {
                cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
                cv::Point2d uv;
                uv = cam_model_.project3dToPixel(pt_cv);

                COLOUR c = GetColour(int((*pt).intensity*255.0), 0, 255);

                cv::circle(img_color, uv, 4, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);

                if (uv.x>=0 && uv.x < img_out.cols && uv.y >= 0 && uv.y < img_out.rows){
                  objective_function += img_out.at<uchar>(uv.y, uv.x)*(*pt).intensity;
                }

              }

              if (save_to_file_){
                savefile << transform.getOrigin()[0] << ", " << transform.getOrigin()[1] << ", " << transform.getOrigin()[2] << ", " << r_mod << ", " << p_mod << ", " << yaw_mod << ", " << objective_function << ", " << endl;
              }

            }
          }
        }
      }
    }
  }

  edges_cloud.reset();

  laserReceived =false;
  cameraReceived = false;

  step_++;

};

void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& velo_cloud){
  ROS_INFO("Velodyne pattern ready!");
  laserReceived = true;
  pcl::PointCloud<Velodyne::Point>::Ptr t_laser_cloud(new pcl::PointCloud<Velodyne::Point>);
  pcl::PointCloud<Velodyne::Point>::Ptr laser_cloud(new pcl::PointCloud<Velodyne::Point>);
  fromROSMsg(*velo_cloud, *laser_cloud);

  Velodyne::addRange(*laser_cloud);

  edges_pointcloud(laser_cloud);

  edges_cloud =  pcl::PointCloud<Velodyne::Point>::Ptr(new pcl::PointCloud<Velodyne::Point>);

  float THRESHOLD = 0.30;
  for (pcl::PointCloud<Velodyne::Point>::iterator pt = laser_cloud->points.begin(); pt < laser_cloud->points.end(); ++pt){
    if(pow(pt->intensity,2)>THRESHOLD){
      edges_cloud->push_back(*pt);
    }
  }

  sensor_msgs::PointCloud2 cloud_ros;
  pcl::toROSMsg(*edges_cloud, cloud_ros);
  cloud_ros.header = velo_cloud->header;
  cloud_pub.publish(cloud_ros);

  t_laser_cloud.reset();
  laser_cloud.reset();

}

void stereo_callback(const sensor_msgs::ImageConstPtr& image_msg){
  ROS_INFO("Camera pattern ready!");
  cameraReceived = true;

  cv::Mat img_in;
  try{
    img_in = cv_bridge::toCvShare(image_msg)->image;
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_gray;
  cv::cvtColor(img_in, img_gray, cv::COLOR_RGB2GRAY);

  cv::Mat img_edge(img_gray.size(), CV_8UC1);;
  edge_max<uchar>(img_gray, img_edge);

  img_out = cv::Mat::zeros(img_edge.size(), CV_8UC1);
  inverse_distance_transformation<uchar,uchar>(img_edge, img_out);
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "levinson");
  ros::NodeHandle nh_("~"); // LOCAL
  image_transport::ImageTransport it_(nh_);

  // Parameters
  nh_.param<std::string>("target_frame", target_frame, "/stereo_camera");
  nh_.param<std::string>("source_frame", source_frame, "/velodyne");
  nh_.param<bool>("listen_to_tf", listen_to_tf_, true);
  nh_.param<bool>("save_to_file", save_to_file_, true);

  laserReceived = false;
  cameraReceived = false;

  image_transport::Subscriber sub = it_.subscribe("image", 1, stereo_callback);
  ros::Subscriber laser_sub = nh_.subscribe ("cloud", 1, laser_callback);
  ros::Subscriber cinfo_sub = nh_.subscribe("camera_info", 1, checkExtrinics);

  cloud_pub = nh_.advertise<sensor_msgs::PointCloud2> ("levinson_cloud", 1);

  step_ = 0;

  ostringstream os;
  os << getenv("HOME") << "/" << "levinson_" << currentDateTime() << ".csv" ;

  if (save_to_file_){
    ROS_INFO("Opening %s", os.str().c_str());
    savefile.open (os.str().c_str());

    savefile << "x, y, z, r, p, y, obj" << endl;
  }

  ros::spin();

  if (save_to_file_) savefile.close();
  return 0;
}
