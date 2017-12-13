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
  disp_masker: Mask the disparity map according to the edges image
*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

class Masker
{
public:
  ros::NodeHandle nh_;

  message_filters::Subscriber<stereo_msgs::DisparityImage> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> mask_sub_;
  ros::Publisher masked_pub_;

  bool isfreeobs_;
  int edges_threshold_;

  typedef message_filters::sync_policies::ExactTime<stereo_msgs::DisparityImage, sensor_msgs::Image> ExSync;
  message_filters::Synchronizer<ExSync> sync;

  Masker():
    nh_("~"),
    image_sub_(nh_, "image", 1),
    mask_sub_(nh_, "mask", 1),
    sync(ExSync(100), image_sub_, mask_sub_)
  {

    masked_pub_ = nh_.advertise<stereo_msgs::DisparityImage>("output", 1);

    nh_.param("isFreeobs", isfreeobs_, false);
    nh_.param("edges_threshold", edges_threshold_, 16);

    sync.registerCallback(boost::bind(&Masker::callback, this, _1, _2));

  }

  void callback(const stereo_msgs::DisparityImageConstPtr& disp, const sensor_msgs::ImageConstPtr& ma)
  {
    cv::Mat mask, binary_mask, output;
    cv_bridge::CvImageConstPtr cv_im;

    try
    {
      cv_im = cv_bridge::toCvShare(disp->image, disp, sensor_msgs::image_encodings::TYPE_32FC1);
      mask = cv_bridge::toCvShare(ma, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("CvBridge failed");
    }

    static cv::Mat disparity32(cv_im->image.rows, cv_im->image.cols, CV_32FC1);
    disparity32 = cv_im->image;

    if (isfreeobs_){
      const static int OBSTACLE_LABEL = 32;
      cv::Mat obs_pattern(mask.rows, mask.cols, CV_8UC1, cv::Scalar(OBSTACLE_LABEL));
      cv::bitwise_and(mask, obs_pattern, binary_mask);
      binary_mask = binary_mask * (255.0/OBSTACLE_LABEL);
    }else{
      cv::threshold(mask, binary_mask, edges_threshold_, 255, 0);
    }

    // Copy input disparity to another DisparityImage variable
    stereo_msgs::DisparityImagePtr copy_disp = boost::make_shared<stereo_msgs::DisparityImage > ();
    copy_disp->valid_window.x_offset         = disp->valid_window.x_offset;
    copy_disp->valid_window.y_offset         = disp->valid_window.y_offset;
    copy_disp->valid_window.width            = disp->valid_window.width;
    copy_disp->valid_window.height           = disp->valid_window.height;
    copy_disp->header                        = disp->header;
    copy_disp->image.header                  = disp->header;
    copy_disp->image.encoding                = sensor_msgs::image_encodings::TYPE_32FC1;
    copy_disp->image.height                  = disp->image.height;
    copy_disp->image.width                   = disp->image.width;
    copy_disp->image.step                    = disp->image.step;
    copy_disp->T                             = disp->T;
    copy_disp->f                             = disp->f;

    copy_disp->min_disparity                 = disp->min_disparity;
    copy_disp->max_disparity                 = disp->max_disparity;
    copy_disp->delta_d                       = disp->delta_d;

    // Create cv::Mat from the copies DisparityImage input
    sensor_msgs::Image& d_image     = copy_disp->image;
    d_image.height                  = disparity32.rows;
    d_image.width                   = disparity32.cols;
    d_image.encoding                = sensor_msgs::image_encodings::TYPE_32FC1;
    d_image.step                    = d_image.width * sizeof(float);

    d_image.data.resize(d_image.step * d_image.height);

    cv::Mat_<float> dmat(d_image.height, d_image.width, (float*)&d_image.data[0], d_image.step);

    // Check data
    ROS_ASSERT(dmat.data == &d_image.data[0]);

    disparity32.copyTo(dmat, binary_mask);

    // Publish obstacle disparity
    masked_pub_.publish(copy_disp);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_masker");

  Masker im;

  ROS_INFO("Ready");
  ros::spin();
}
