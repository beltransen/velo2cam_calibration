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

#ifndef velo2cam_utils_H
#define velo2cam_utils_H

#define PCL_NO_PRECOMPILE
#define DEBUG 0

#include <vector>
#include <cmath>
#include <unistd.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

using namespace std;

static const int RINGS_COUNT = 16; // TODO AS FUNCTION PARAM

namespace Velodyne {
  struct Point
  {
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t ring; ///< laser ring number
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
  }EIGEN_ALIGN16;

  void addRange(pcl::PointCloud<Velodyne::Point> & pc){
    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
    }
  }

  vector<vector<Point*> > getRings(pcl::PointCloud<Velodyne::Point> & pc)
  {
    vector<vector<Point*> > rings(RINGS_COUNT);
    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      rings[pt->ring].push_back(&(*pt));
    }
    return rings;
  }

  // all intensities to range min-max
  void normalizeIntensity(pcl::PointCloud<Point> & pc, float minv, float maxv)
  {
    float min_found = INFINITY;
    float max_found = -INFINITY;

    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      max_found = max(max_found, pt->intensity);
      min_found = min(min_found, pt->intensity);
    }

    for (pcl::PointCloud<Point>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
    {
      pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (maxv - minv) + minv;
    }
  }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
  Velodyne::Point, (float, x, x) (float, y, y) (float, z, z)
  (float, intensity, intensity) (uint16_t, ring, ring) (float, range, range));

void sortPatternCentersXY(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, vector<pcl::PointXYZ> &v){
  double avg_x = 0, avg_y = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    avg_x += (*it).x;
    avg_y += (*it).y;
  }

  pcl::PointXYZ center;
  center.x = avg_x/4.;
  center.y = avg_y/4.;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    double x_dif = (*it).x - center.x;
    double y_dif = (*it).y - center.y;

    if(x_dif < 0 && y_dif < 0){
      v[0] = (*it);
    }else if(x_dif > 0 && y_dif < 0){
      v[1] = (*it);
    }else if(x_dif > 0 && y_dif > 0){
      v[2] = (*it);
    }else{
      v[3] = (*it);
    }
  }
}

void sortPatternCentersYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, vector<pcl::PointXYZ> &v){
  double avg_y = 0, avg_z = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    avg_y += (*it).y;
    avg_z += (*it).z;
  }

  pcl::PointXYZ center;
  center.y = avg_y/4.;
  center.z = avg_z/4.;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    double y_dif = (*it).y - center.y;
    double z_dif = (*it).z - center.z;

    if(y_dif > 0 && z_dif > 0){
      v[0] = (*it);
    }else if(y_dif < 0 && z_dif > 0){
      v[1] = (*it);
    }else if(y_dif < 0 && z_dif < 0){
      v[2] = (*it);
    }else{
      v[3] = (*it);
    }
  }
}

void colourCenters(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, pcl::PointCloud<pcl::PointXYZI>::Ptr coloured){
  double avg_x = 0, avg_y = 0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    avg_x += (*it).x;
    avg_y += (*it).y;
  }

  pcl::PointXYZ center;
  center.x = avg_x/4.;
  center.y = avg_y/4.;

  for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pc->points.begin(); it<pc->points.end(); it++){
    double x_dif = (*it).x - center.x;
    double y_dif = (*it).y - center.y;


    pcl::PointXYZI cc;
    cc.x = (*it).x;
    cc.y = (*it).y;
    cc.z = (*it).z;

    if(x_dif < 0 && y_dif < 0){
      cc.intensity = 0;
    }else if(x_dif > 0 && y_dif < 0){
      cc.intensity = 0.3;
    }else if(x_dif > 0 && y_dif > 0){
      cc.intensity = 0.6;
    }else{
      cc.intensity = 1;
    }
    coloured->push_back(cc);
  }
}

void getCenterClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud,
  double cluster_tolerance = 0.10, int min_cluster_size = 15, int max_cluster_size = 200, bool verbosity=true){

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;
  euclidean_cluster.setClusterTolerance (cluster_tolerance);
  euclidean_cluster.setMinClusterSize (min_cluster_size);
  euclidean_cluster.setMaxClusterSize (max_cluster_size);
  euclidean_cluster.setSearchMethod (tree);
  euclidean_cluster.setInputCloud (cloud_in);
  euclidean_cluster.extract (cluster_indices);

  if(DEBUG && verbosity) cout << cluster_indices.size() << " clusters found from " << cloud_in->points.size() << " points in cloud" << endl;

  for (std::vector<pcl::PointIndices>::iterator it=cluster_indices.begin(); it<cluster_indices.end(); it++) {
    float accx = 0., accy = 0., accz = 0.;
    for(vector<int>::iterator it2=it->indices.begin(); it2<it->indices.end(); it2++){
      accx+=cloud_in->at(*it2).x;
      accy+=cloud_in->at(*it2).y;
      accz+=cloud_in->at(*it2).z;
    }
    // Compute and add center to clouds
    pcl::PointXYZ center;
    center.x =  accx/it->indices.size();
    center.y =  accy/it->indices.size();
    center.z =  accz/it->indices.size();
    centers_cloud->push_back(center);
  }
}

Eigen::Affine3f getRotationMatrix(Eigen::Vector3f source, Eigen::Vector3f target){
  Eigen::Vector3f rotation_vector = target.cross(source);
  rotation_vector.normalize();
  double theta = acos(source[2]/sqrt( pow(source[0],2)+ pow(source[1],2) + pow(source[2],2)));

  if(DEBUG) ROS_INFO("Rot. vector: (%lf %lf %lf) / Angle: %lf", rotation_vector[0], rotation_vector[1], rotation_vector[2], theta);

  Eigen::Matrix3f rotation = Eigen::AngleAxis<float>(theta, rotation_vector) * Eigen::Scaling(1.0f);
  Eigen::Affine3f rot(rotation);
  return rot;
}

#endif
