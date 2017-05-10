# velo2cam_calibration [![Build Status](http://build.ros.org/job/Kdev__velo2cam_calibration__ubuntu_xenial_amd64/8/badge/icon)](http://build.ros.org/job/Kdev__velo2cam_calibration__ubuntu_xenial_amd64/8/)

## Overview ###
The *velo2cam_calibration* software implements an Automatic Calibration algorithm for Lidar-Stereo camera setups. This software is provided as a ROS package.

Note: Package developed at Intelligent Systems Laboratory (http://www.uc3m.es/islab), Universidad Carlos III de Madrid.

![gazebo screenshot](screenshots/velo2cam_calibration_setup.png)

## 1. Nodes ##
### 1.1 stereo_pattern ###
#### Subscribed Topics ####
cloud1 (sensor_msgs/PointCloud2)
   Laser pointcloud
#### Published ####
#### Parameters ####
### 1.2 laser_pattern ###
#### Subscribed Topics ####
cloud1 (sensor_msgs/PointCloud2)
   Laser pointcloud
#### Published Topics ####
#### Parameters ####
### 1.3 velo2cam_calibration ###
#### Subscribed Topics ####
cloud1 (velo2cam_calibration::ClusterCentroids)
cloud2 (velo2cam_calibration::ClusterCentroids)
#### Published Topics ####
tf relating both sensors
#### Parameters ####

## 3. Usage ##
Some sample .launch files are provided in this package. The simplest way to launch the algorithm is by running the three main ROS nodes as follows:

```roslaunch velo2cam_calibration lidar_pattern.launch```

```roslaunch velo2cam_calibration stereo_pattern.launch```

```roslaunch velo2cam_calibration velo2cam_calibration.launch```

Note: In order to test the algorithm with a proper ground truth, a simulator environment in Gazebo is provided at *https://github.com/beltransen/velo2cam_gazebo*

## 4. Citation ##
Paper *"Automatic Extrinsic Calibration for Lidar-Stereo Vehicle Sensor Setups"* submitted to *International Conference on Intelligent Transportation Systems (ITSC) 2017*.

For citation details, please contact Jorge Beltran (jbeltran AT ing.uc3m DOT es) or Carlos Guindel (cguindel AT ing.uc3m DOT es).
