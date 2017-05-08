# velo2cam_calibration [![Build Status](http://build.ros.org/job/Kdev__velo2cam_calibration__ubuntu_xenial_amd64/8/badge/icon)](http://build.ros.org/job/Kdev__velo2cam_calibration__ubuntu_xenial_amd64/8/)
Automatic Calibration algorithm for Lidar-Stereo camera. ROS Package.

## Package summary ###
The *velo2cam_calibration* package implements the algorithm for calibrating relative position between a laser (velodyne) and a stereo camera. 

![gazebo screenshot](screenshots/velo2cam_calibration_setup.png)

## 1. Overview ##

## 2. Nodes ##
### 2.1 stereo_pattern ###
#### Subscribed Topics #### 
cloud1 (sensor_msgs/PointCloud2)
   Laser pointcloud
#### Published ####
#### Parameters ####
### 2.2 laser_pattern ###
#### Subscribed Topics ####
cloud1 (sensor_msgs/PointCloud2)
   Laser pointcloud
#### Published Topics ####
#### Parameters ####
### 2.3 velo2cam_calibration ###
#### Subscribed Topics ####
cloud1 (velo2cam_calibration::ClusterCentroids)
cloud2 (velo2cam_calibration::ClusterCentroids)
#### Published Topics ####
tf relating both sensors
#### Parameters ####
