# velo2cam_calibration [![Build Status](http://build.ros.org/job/Kdev__velo2cam_calibration__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__velo2cam_calibration__ubuntu_xenial_amd64/)

The *velo2cam_calibration* software implements a state-of-the-art automatic calibration algorithm for pair of sensors composed of LiDAR and camera devices in any possible combination \[1\]. This software is provided as a ROS package.

Package developed at [Intelligent Systems Laboratory](http://www.uc3m.es/islab), Universidad Carlos III de Madrid.

![real results](screenshots/real_results.png)

## Setup ##
To install this ROS package:
1. Clone the repository into your *catkin_ws/src/* folder.
2. Install run dependencies: ```sudo apt-get install ros-<distro>-opencv-apps```
3. Build your workspace [as usual](http://wiki.ros.org/ROS/Tutorials/BuildingPackages).

## Usage ##
See [HOWTO.md](HOWTO.md) for detailed instructions on how to use this software.

To test the algorithm in a virtual environment, you can launch any of the calibration scenarios included in our [Gazebo validation suite](https://github.com/beltransen/velo2cam_gazebo).

## Calibration target ##
The following picture shows a possible embodiment of the proposed calibration target used by this algorithm and its corresponding dimensional drawing.

![calibration target](screenshots/calibration_target_real_scheme_journal.png)

**Note:** Other size may be used for convenience. If so, please configure node parameters accordingly.

## Citation ##
If you use this work in your research, please consider citing the following paper:

[1] Beltrán, J., Guindel, C., and García, F. (2021). [Automatic Extrinsic Calibration Method for LiDAR and Camera Sensor Setups](https://arxiv.org/abs/2101.04431). arXiv:2101.04431 [cs.RO]. Submitted to IEEE Transactions on Intelligent Transportation Systems.

A previous version of this tool is available [here](https://github.com/beltransen/velo2cam_calibration/tree/v1.0) and was described on this paper: 

[2] Guindel, C., Beltrán, J., Martín, D., and García, F. (2017).  [Automatic Extrinsic Calibration for Lidar-Stereo Vehicle Sensor Setup](https://arxiv.org/abs/1705.04085). *IEEE International Conference on Intelligent Transportation Systems (ITSC), 674–679*.