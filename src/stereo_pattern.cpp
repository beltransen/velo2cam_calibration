/*
  velo2cam_calibration - Automatic calibration algorithm for extrinsic
  parameters of a stereo camera and a velodyne Copyright (C) 2017-2021 Jorge
  Beltran, Carlos Guindel

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

#define TARGET_NUM_CIRCLES 4

#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <velo2cam_calibration/StereoConfig.h>
#include <velo2cam_calibration/ClusterCentroids.h>
#include <velo2cam_utils.h>

using namespace std;
using namespace sensor_msgs;

int images_proc_ = 0, images_used_ = 0;

double delta_width_circles_, delta_height_circles_;
double circle_threshold_;
double plane_distance_inliers_;
double target_radius_tolerance_;
double cluster_tolerance_;
double min_cluster_factor_;
int min_centers_found_;
bool WARMUP_DONE = false;
bool skip_warmup_;
bool save_to_file_;
std::ofstream savefile;

ros::Publisher inliers_pub;
ros::Publisher coeff_pub;
ros::Publisher plane_edges_pub;
ros::Publisher xy_pattern_pub;
ros::Publisher cumulative_pub;
ros::Publisher final_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;

std_msgs::Header header_;

void callback(const PointCloud2::ConstPtr &camera_cloud,
              const pcl_msgs::ModelCoefficients::ConstPtr &cam_plane_coeffs) {
  if (DEBUG) ROS_INFO("[Stereo] Processing image...");

  images_proc_++;

  header_ = camera_cloud->header;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*camera_cloud, *cam_cloud);

  // 1.FILTER THE EDGES-CLOUD ACCORDING TO THE DETECTED PLANE
  // Segment inliers
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = cam_plane_coeffs->values[0];
  coefficients_v(1) = cam_plane_coeffs->values[1];
  coefficients_v(2) = cam_plane_coeffs->values[2];
  coefficients_v(3) = cam_plane_coeffs->values[3];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_plane_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr dit(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cam_cloud));
  std::vector<int> plane_inliers;
  dit->selectWithinDistance(coefficients_v, plane_distance_inliers_,
                            plane_inliers);
  pcl::copyPointCloud<pcl::PointXYZ>(*cam_cloud, plane_inliers,
                                     *cam_plane_cloud);

  // Publish plane as "plane_edges_cloud"
  if (DEBUG) {
    PointCloud2 plane_edges_ros;
    pcl::toROSMsg(*cam_plane_cloud, plane_edges_ros);
    plane_edges_ros.header = camera_cloud->header;
    plane_edges_pub.publish(plane_edges_ros);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // 2.ROTATE CLOUD TO FACE PATTERN PLANE
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3f xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;

  floor_plane_normal_vector[0] = cam_plane_coeffs->values[0];
  floor_plane_normal_vector[1] = cam_plane_coeffs->values[1];
  floor_plane_normal_vector[2] = cam_plane_coeffs->values[2];

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cam_plane_cloud, *cam_plane_cloud, indices);

  Eigen::Affine3f rotation =
      getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*cam_plane_cloud, *xy_cloud, rotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3) / coefficients_v(2));
  aux_cloud->push_back(aux_point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr auxrotated_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);

  double zcoord_xyplane = auxrotated_cloud->at(0).z;

  // Force pattern points to belong to computed plane
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = xy_cloud->points.begin();
       pt < xy_cloud->points.end(); ++pt) {
    pt->z = zcoord_xyplane;
  }

  // Publishing "xy_pattern" (pattern transformed to be aligned with XY)
  if (DEBUG) {
    PointCloud2 xy_pattern_ros;
    pcl::toROSMsg(*xy_cloud, xy_pattern_ros);
    xy_pattern_ros.header = camera_cloud->header;
    xy_pattern_pub.publish(xy_pattern_ros);
  }

  // 3.FIND CIRCLES
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setModelType(pcl::SACMODEL_CIRCLE2D);
  seg.setDistanceThreshold(circle_threshold_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(1000);
  seg.setRadiusLimits(TARGET_RADIUS - target_radius_tolerance_,
                      TARGET_RADIUS + target_radius_tolerance_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::vector<float>> found_centers;

  do {
    seg.setInputCloud(xy_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      if (found_centers.size() < 1) {
        ROS_WARN(
            "[Stereo] Could not estimate a circle model for the given "
            "dataset.");
      }
      break;
    } else {
      if (DEBUG)
        ROS_INFO("[Stereo] Found circle: %lu inliers", inliers->indices.size());
    }

    // Extract the inliers
    extract.setInputCloud(xy_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*circle_cloud);

    // Add center point to cloud (only if it makes sense)
    pcl::PointXYZ center;
    center.x = *coefficients->values.begin();
    center.y = *(coefficients->values.begin() + 1);
    center.z = zcoord_xyplane;

    std::vector<float> found_center;
    found_center.push_back(center.x);
    found_center.push_back(center.y);
    found_center.push_back(center.z);
    found_centers.push_back(found_center);

    // Remove inliers from pattern cloud to find next circle
    extract.setNegative(true);
    extract.filter(*temp_cloud);
    xy_cloud.swap(temp_cloud);

  } while (xy_cloud->points.size() > 3);

  if (found_centers.size() <
      min_centers_found_) {  // Usually min_centers_found_ = TARGET_NUM_CIRCLES
    // Exit 1: centers not found
    ROS_WARN("Not enough centers: %ld", found_centers.size());
    return;
  }

  /**
    4. GEOMETRIC CONSISTENCY CHECK
    At this point, circles' center candidates have been computed
  (found_centers). Now we need to select the set of 4 candidates that best fit
  the calibration target geometry. To that end, the following steps are
  followed: 1) Create a cloud with 4 points representing the exact geometry of
  the calibration target 2) For each possible set of 4 points: compute
  similarity score 3) Rotate back the candidates with the highest score to their
  original position in the cloud, and add them to cumulative cloud
  **/
  std::vector<std::vector<int>> groups;
  comb(found_centers.size(), TARGET_NUM_CIRCLES, groups);
  double groups_scores[groups.size()];  // -1: invalid; 0-1 normalized score
  for (int i = 0; i < groups.size(); ++i) {
    std::vector<pcl::PointXYZ> candidates;
    // Build candidates set
    for (int j = 0; j < groups[i].size(); ++j) {
      pcl::PointXYZ center;
      center.x = found_centers[groups[i][j]][0];
      center.y = found_centers[groups[i][j]][1];
      center.z = found_centers[groups[i][j]][2];
      candidates.push_back(center);
    }

    // Compute candidates score
    Square square_candidate(candidates, delta_width_circles_,
                            delta_height_circles_);
    groups_scores[i] = square_candidate.is_valid()
                           ? 1.0
                           : -1;  // -1 when it's not valid, 1 otherwise
  }

  int best_candidate_idx = -1;
  double best_candidate_score = -1;
  for (int i = 0; i < groups.size(); ++i) {
    if (best_candidate_score == 1 && groups_scores[i] == 1) {
      // Exit 2: Several candidates fit target's geometry
      ROS_ERROR(
          "[Stereo] More than one set of candidates fit target's geometry. "
          "Please, make sure your parameters are well set. Exiting callback");
      return;
    }
    if (groups_scores[i] > best_candidate_score) {
      best_candidate_score = groups_scores[i];
      best_candidate_idx = i;
    }
  }

  if (best_candidate_idx == -1) {
    // Exit 3: No candidates fit target's geometry
    ROS_WARN(
        "[Stereo] Unable to find a candidate set that matches target's "
        "geometry");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_back_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int j = 0; j < groups[best_candidate_idx].size(); ++j) {
    pcl::PointXYZ point;
    point.x = found_centers[groups[best_candidate_idx][j]][0];
    point.y = found_centers[groups[best_candidate_idx][j]][1];
    point.z = found_centers[groups[best_candidate_idx][j]][2];

    pcl::PointXYZ center_rotated_back =
        pcl::transformPoint(point, rotation.inverse());
    center_rotated_back.z =
        (-cam_plane_coeffs->values[0] * center_rotated_back.x -
         cam_plane_coeffs->values[1] * center_rotated_back.y -
         cam_plane_coeffs->values[3]) /
        cam_plane_coeffs->values[2];

    rotated_back_cloud->push_back(center_rotated_back);
    cumulative_cloud->push_back(center_rotated_back);
  }

  if (save_to_file_) {
    std::vector<pcl::PointXYZ> sorted_centers;
    sortPatternCenters(rotated_back_cloud, sorted_centers);
    for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
         it < sorted_centers.end(); ++it) {
      savefile << it->x << ", " << it->y << ", " << it->z << ", ";
    }
  }

  // Publishing "cumulative_cloud" (centers found from the beginning)
  if (DEBUG) {
    PointCloud2 cumulative_ros;
    pcl::toROSMsg(*cumulative_cloud, cumulative_ros);
    cumulative_ros.header = camera_cloud->header;
    cumulative_pub.publish(cumulative_ros);
  }

  pcl_msgs::PointIndices p_ind;

  pcl_conversions::moveFromPCL(*inliers, p_ind);
  p_ind.header = camera_cloud->header;

  pcl_msgs::ModelCoefficients m_coeff;

  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = camera_cloud->header;

  if (DEBUG) {
    inliers_pub.publish(p_ind);
    coeff_pub.publish(m_coeff);
  }

  images_used_++;
  if (DEBUG) {
    ROS_INFO("[Stereo] %d/%d frames: %ld pts in cloud", images_used_,
             images_proc_, cumulative_cloud->points.size());
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Compute circles centers
  if (!WARMUP_DONE) {  // Compute clusters from detections in the latest frame
    getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_, 1, 1);
  } else {  // Use cumulative information from previous frames
    getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_,
                      min_cluster_factor_ * images_used_, images_used_);
    if (final_cloud->points.size() > TARGET_NUM_CIRCLES) {
      getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_,
                        3.0 * images_used_ / 4.0, images_used_);
    }
  }

  // Exit 4: clustering failed
  if (final_cloud->points.size() == TARGET_NUM_CIRCLES) {
    if (save_to_file_) {
      std::vector<pcl::PointXYZ> sorted_centers;
      sortPatternCenters(final_cloud, sorted_centers);
      for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
           it < sorted_centers.end(); ++it) {
        savefile << it->x << ", " << it->y << ", " << it->z << ", ";
      }
      savefile << cumulative_cloud->width;
    }

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

  if (save_to_file_) {
    savefile << endl;
  }

  // Clear cumulative cloud during warm-up phase
  if (!WARMUP_DONE) {
    cumulative_cloud->clear();
    images_proc_ = 0;
    images_used_ = 0;
  }
}

void param_callback(velo2cam_calibration::StereoConfig &config,
                    uint32_t level) {
  circle_threshold_ = config.circle_threshold;
  ROS_INFO("[Stereo] New circle threshold: %f", circle_threshold_);
}

void warmup_callback(const std_msgs::Empty::ConstPtr &msg) {
  WARMUP_DONE = !WARMUP_DONE;
  if (WARMUP_DONE) {
    ROS_INFO("[Stereo] Warm up done, pattern detection started");
  } else {
    ROS_INFO("[Stereo] Detection stopped. Warm up mode activated");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_pattern");
  ros::NodeHandle nh;        // GLOBAL
  ros::NodeHandle nh_("~");  // LOCAL

  message_filters::Subscriber<PointCloud2> camera_cloud_sub_;
  message_filters::Subscriber<pcl_msgs::ModelCoefficients>
      cam_plane_coeffs_sub_;

  camera_cloud_sub_.subscribe(nh_, "cloud2", 1);
  cam_plane_coeffs_sub_.subscribe(nh_, "cam_plane_coeffs", 1);

  if (DEBUG) {
    inliers_pub = nh_.advertise<pcl_msgs::PointIndices>("inliers", 1);
    coeff_pub = nh_.advertise<pcl_msgs::ModelCoefficients>("model", 1);
    plane_edges_pub = nh_.advertise<PointCloud2>("plane_edges_cloud", 1);
    xy_pattern_pub = nh_.advertise<PointCloud2>("xy_pattern", 1);
    cumulative_pub = nh_.advertise<PointCloud2>("cumulative_cloud", 1);
  }

  final_pub =
      nh_.advertise<velo2cam_calibration::ClusterCentroids>("centers_cloud", 1);

  cumulative_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  typedef message_filters::sync_policies::ExactTime<PointCloud2,
                                                    pcl_msgs::ModelCoefficients>
      ExSync;
  message_filters::Synchronizer<ExSync> sync_(ExSync(10), camera_cloud_sub_,
                                              cam_plane_coeffs_sub_);
  sync_.registerCallback(boost::bind(&callback, _1, _2));

  string csv_name;

  nh.param("delta_width_circles", delta_width_circles_, 0.5);
  nh.param("delta_height_circles", delta_height_circles_, 0.4);
  nh_.param("plane_distance_inliers", plane_distance_inliers_, 0.1);
  nh_.param("target_radius_tolerance", target_radius_tolerance_, 0.01);
  nh_.param("min_centers_found", min_centers_found_, TARGET_NUM_CIRCLES);
  nh_.param("cluster_tolerance", cluster_tolerance_, 0.05);
  nh_.param("min_cluster_factor", min_cluster_factor_, 0.5);
  nh_.param("skip_warmup", skip_warmup_, false);
  nh_.param("save_to_file", save_to_file_, false);
  nh_.param("csv_name", csv_name,
            "stereo_pattern_" + currentDateTime() + ".csv");

  dynamic_reconfigure::Server<velo2cam_calibration::StereoConfig> server;
  dynamic_reconfigure::Server<velo2cam_calibration::StereoConfig>::CallbackType
      f;
  f = boost::bind(param_callback, _1, _2);
  server.setCallback(f);

  ros::Subscriber warmup_sub =
      nh.subscribe("warmup_switch", 1, warmup_callback);

  if (skip_warmup_) {
    ROS_WARN("Skipping warmup");
    WARMUP_DONE = true;
  }

  // Just for statistics
  if (save_to_file_) {
    ostringstream os;
    os << getenv("HOME") << "/v2c_experiments/" << csv_name;
    if (save_to_file_) {
      if (DEBUG) ROS_INFO("Opening %s", os.str().c_str());
      savefile.open(os.str().c_str());
      savefile << "det1_x, det1_y, det1_z, det2_x, det2_y, det2_z, det3_x, "
                  "det3_y, det3_z, det4_x, det4_y, det4_z, cent1_x, cent1_y, "
                  "cent1_z, cent2_x, cent2_y, cent2_z, cent3_x, cent3_y, "
                  "cent3_z, cent4_x, cent4_y, cent4_z, it"
               << endl;
    }
  }

  ros::spin();
  return 0;
}
