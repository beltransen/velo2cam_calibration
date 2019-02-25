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
  velo2cam_calibration: Perform the registration step
*/

#define PCL_NO_PRECOMPILE

#include "velo2cam_calibration/ClusterCentroids.h"

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <ctime>
#include "tinyxml.h"

#ifdef TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#else
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#endif


#include "velo2cam_utils.h"

using namespace std;
using namespace sensor_msgs;

ros::Publisher aux_pub, aux2_pub;
ros::Publisher t_pub;
ros::Publisher clusters_c, clusters_l;
int nFrames;
bool laserReceived, cameraReceived;

pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud, camera_cloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr ilaser_cloud, icamera_cloud;
std::vector<pcl::PointXYZ> lv(4), cv(4);

tf::StampedTransform tf_velodyne_camera;

typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;

tf::Transform transf;

std::vector< std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> > > laser_buffer;
std::vector< std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> > > cam_buffer;

bool sync_iterations;
bool save_to_file_;
bool publish_tf_;

long int laser_count, cam_count;

std::ofstream savefile;

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

void calibrateExtrinsics(int seek_iter = -1){
  // ROS_INFO("Hey man, keep calm... I'm calibrating your sensors...");

  std::vector<pcl::PointXYZ> local_lv, local_cv;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_laser_cloud, local_camera_cloud;
  pcl::PointCloud<pcl::PointXYZ> local_l_cloud, local_c_cloud;

  int used_stereo, used_laser;

  if (seek_iter>0){
    if(DEBUG) ROS_INFO("Seeking %d iterations", seek_iter);
    if(DEBUG) ROS_INFO("Last cam: %d, last laser: %d", std::get<0>(cam_buffer.back()),std::get<0>(laser_buffer.back()));
    auto it = std::find_if(cam_buffer.begin(), cam_buffer.end(), [&seek_iter](const std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >& e) {return std::get<0>(e) == seek_iter;});
    if (it == cam_buffer.end()) {
      ROS_WARN("Could not sync cam");
      return;
    }

    auto it2 = std::find_if(laser_buffer.begin(), laser_buffer.end(), [&seek_iter](const std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >& e) {return std::get<0>(e) == seek_iter;});
    if (it2 == laser_buffer.end()) {
      ROS_WARN("Could not sync laser");
      return;
    }

    used_stereo = std::get<1>(*it);
    used_laser = std::get<1>(*it2);

    local_cv = std::get<3>(*it);
    local_c_cloud = std::get<2>(*it);
    local_camera_cloud = local_c_cloud.makeShared();

    local_lv = std::get<3>(*it2);
    local_l_cloud = std::get<2>(*it2);
    local_laser_cloud = local_l_cloud.makeShared();
    ROS_INFO("Synchronizing cluster centroids");
  }else{
    local_lv = lv;
    local_cv = cv;
    local_laser_cloud = laser_cloud;
    local_camera_cloud = camera_cloud;
  }

  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*local_camera_cloud, ros_cloud);
  ros_cloud.header.frame_id = "stereo";
  clusters_c.publish(ros_cloud);

  pcl::toROSMsg(*local_laser_cloud, ros_cloud);
  ros_cloud.header.frame_id = "velodyne";
  clusters_l.publish(ros_cloud);

  Vector12d las_12;
  las_12 <<   local_lv[0].x,
  local_lv[0].y,
  local_lv[0].z,
  local_lv[1].x,
  local_lv[1].y,
  local_lv[1].z,
  local_lv[2].x,
  local_lv[2].y,
  local_lv[2].z,
  local_lv[3].x,
  local_lv[3].y,
  local_lv[3].z;

  Vector12d cam_12;
  cam_12 <<   local_cv[0].x,
  local_cv[0].y,
  local_cv[0].z,
  local_cv[1].x,
  local_cv[1].y,
  local_cv[1].z,
  local_cv[2].x,
  local_cv[2].y,
  local_cv[2].z,
  local_cv[3].x,
  local_cv[3].y,
  local_cv[3].z;

  Vector12d diff_12;
  diff_12 = las_12-cam_12;

  Eigen::MatrixXd matrix_transl(12,3);
  matrix_transl <<    1, 0, 0, 0, 1, 0, 0, 0, 1,
  1, 0, 0, 0, 1, 0, 0, 0, 1,
  1, 0, 0, 0, 1, 0, 0, 0, 1,
  1, 0, 0, 0, 1, 0, 0, 0, 1;

  Eigen::Vector3d x;
  x = matrix_transl.colPivHouseholderQr().solve(diff_12);

  // cout << "The least-squares solution is:\n"
  //     << x << endl;

  Eigen::Matrix4f Tm;
  Tm <<   1, 0, 0, x[0],
  0, 1, 0, x[1],
  0, 0, 1, x[2],
  0, 0, 0, 1;

  if(DEBUG) ROS_INFO("Step 1: Translation");
  if(DEBUG) cout << Tm << endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr translated_pc (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*local_camera_cloud, *translated_pc, Tm);

  pcl::toROSMsg(*translated_pc, ros_cloud);
  ros_cloud.header.frame_id = "velodyne";
  t_pub.publish(ros_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(translated_pc);
  icp.setInputTarget(local_laser_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  icp.setMaxCorrespondenceDistance(0.2);
  icp.setMaximumIterations(1000);
  if (icp.hasConverged()){
    if(DEBUG) ROS_INFO("ICP Converged. Score: %lf", icp.getFitnessScore());
  }else{
    ROS_WARN("ICP failed to converge");
    return;
  }
  if(DEBUG) ROS_INFO("Step 2. ICP Transformation:");
  if(DEBUG) cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  Eigen::Matrix4f final_trans = transformation * Tm;

  tf::Matrix3x3 tf3d;
  tf3d.setValue(final_trans(0,0), final_trans(0,1), final_trans(0,2),
  final_trans(1,0), final_trans(1,1), final_trans(1,2),
  final_trans(2,0), final_trans(2,1), final_trans(2,2));

  if(DEBUG) ROS_INFO("Final Transformation");
  if(DEBUG) cout << final_trans << endl;

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  #ifdef TF2

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "velodyne";
  transformStamped.child_frame_id = "stereo";
  transformStamped.transform.translation.x = final_trans(0,3);
  transformStamped.transform.translation.y = final_trans(1,3);
  transformStamped.transform.translation.z = final_trans(2,3);
  transformStamped.transform.rotation.x = tfqt.x();
  transformStamped.transform.rotation.y = tfqt.y();
  transformStamped.transform.rotation.z = tfqt.z();
  transformStamped.transform.rotation.w = tfqt.w();

  br.sendTransform(transformStamped);

  #else

  tf::Vector3 origin;
  origin.setValue(final_trans(0,3),final_trans(1,3),final_trans(2,3));

  transf.setOrigin(origin);
  transf.setRotation(tfqt);

  #endif

  static tf::TransformBroadcaster br;
  tf_velodyne_camera = tf::StampedTransform(transf, ros::Time::now(), "velodyne", "stereo");
  if (publish_tf_) br.sendTransform(tf_velodyne_camera);

  tf::Transform inverse = tf_velodyne_camera.inverse();
  double roll, pitch, yaw;
  double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(), zt = inverse.getOrigin().getZ();
  inverse.getBasis().getRPY(roll, pitch, yaw);

  if (save_to_file_){
    savefile << seek_iter << ", " << xt << ", " << yt << ", " << zt << ", " << roll << ", " << pitch << ", " << yaw << ", " << used_laser << ", " << used_stereo << endl;
  }

  ROS_INFO("[V2C] Calibration result:");
  ROS_INFO("x=%.4f y=%.4f z=%.4f",xt,yt,zt);
  ROS_INFO("roll=%.4f, pitch=%.4f, yaw=%.4f",roll,pitch,yaw);
  // ROS_INFO("Translation matrix");
  // ROS_INFO("%.4f, %.4f, %.4f",inverse.getBasis()[0][0],inverse.getBasis()[0][1],inverse.getBasis()[0][2]);
  // ROS_INFO("%.4f, %.4f, %.4f",inverse.getBasis()[1][0],inverse.getBasis()[1][1],inverse.getBasis()[1][2]);
  // ROS_INFO("%.4f, %.4f, %.4f",inverse.getBasis()[2][0],inverse.getBasis()[2][1],inverse.getBasis()[2][2]);

  laserReceived = false;
  cameraReceived = false;
}

void laser_callback(const velo2cam_calibration::ClusterCentroids::ConstPtr velo_centroids){
  // ROS_INFO("Velodyne pattern ready!");
  laserReceived = true;

  fromROSMsg(velo_centroids->cloud, *laser_cloud);

  sortPatternCentersYZ(laser_cloud, lv);
  colourCenters(laser_cloud, ilaser_cloud);

  laser_buffer.push_back(std::tuple<int,int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >(velo_centroids->total_iterations, velo_centroids->cluster_iterations, *laser_cloud,lv));
  laser_count = velo_centroids->total_iterations;

  if(DEBUG) ROS_INFO("[V2C] LASER");

  for(vector<pcl::PointXYZ>::iterator it=lv.begin(); it<lv.end(); ++it){
    if (DEBUG) cout << "l" << it - lv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
  }

  if (sync_iterations){
    if(cam_count >= laser_count){
      calibrateExtrinsics(laser_count);
      return;
    }else{
      if (tf_velodyne_camera.frame_id_ != ""){
        static tf::TransformBroadcaster br;
        tf_velodyne_camera.stamp_ = ros::Time::now();
        if (publish_tf_) br.sendTransform(tf_velodyne_camera);
        return;
      }
    }
  }

  if(laserReceived && cameraReceived){
    calibrateExtrinsics();
  }else{
    static tf::TransformBroadcaster br;
    tf_velodyne_camera.stamp_ = ros::Time::now();
    if (publish_tf_) br.sendTransform(tf_velodyne_camera);
  }
}

void stereo_callback(velo2cam_calibration::ClusterCentroids::ConstPtr image_centroids){
  // if(DEBUG) ROS_INFO("Camera pattern ready!");

#ifdef TF2

  //TODO: adapt to ClusterCentroids

  PointCloud2 xy_image_cloud;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("stereo", "stereo_camera",
                             ros::Time(0), ros::Duration(20));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
  tf2::doTransform (*image_cloud, xy_image_cloud, transformStamped);
  fromROSMsg(xy_image_cloud, *camera_cloud);

#else

  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_camera_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  fromROSMsg(image_centroids->cloud, *xy_camera_cloud);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.waitForTransform("stereo", "stereo_camera", ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform ("stereo", "stereo_camera", ros::Time(0), transform);
  }catch (tf::TransformException& ex) {
    ROS_WARN("TF exception:\n%s", ex.what());
    return;
  }

  tf::Transform inverse = transform.inverse();
  double roll, pitch, yaw;
  inverse.getBasis().getRPY(roll, pitch, yaw);

  pcl_ros::transformPointCloud (*xy_camera_cloud, *camera_cloud, transform);

#endif

  cameraReceived = true;

  sortPatternCentersYZ(camera_cloud, cv);
  colourCenters(camera_cloud, icamera_cloud);

  cam_buffer.push_back(std::tuple<int, int,pcl::PointCloud<pcl::PointXYZ>, std::vector<pcl::PointXYZ> >(image_centroids->total_iterations,image_centroids->cluster_iterations,*camera_cloud,cv));
  cam_count = image_centroids->total_iterations;

  if(DEBUG) ROS_INFO("[V2C] CAMERA");

  for(vector<pcl::PointXYZ>::iterator it=cv.begin(); it<cv.end(); ++it){
    if (DEBUG) cout << "c" << it - cv.begin() << "="<< "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]"<<endl;
  }

  if (sync_iterations){
    if(laser_count >= cam_count){
      calibrateExtrinsics(cam_count);
      return;
    }else{
      if (tf_velodyne_camera.frame_id_ != ""){
        static tf::TransformBroadcaster br;
        tf_velodyne_camera.stamp_ = ros::Time::now();
        if (publish_tf_) br.sendTransform(tf_velodyne_camera);
        return;
      }
    }
  }

  if(laserReceived && cameraReceived){
    if(DEBUG) ROS_INFO("[V2C] Calibrating...");
    calibrateExtrinsics();
  }else{
    static tf::TransformBroadcaster br;
    tf_velodyne_camera.stamp_ = ros::Time::now();
    if (publish_tf_) br.sendTransform(tf_velodyne_camera);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "velo2cam_calibration");
  ros::NodeHandle nh_("~"); // LOCAL

  nh_.param<bool>("sync_iterations", sync_iterations, false);
  nh_.param<bool>("save_to_file", save_to_file_, false);
  nh_.param<bool>("publish_tf", publish_tf_, true);

  static tf::TransformBroadcaster br;
  tf_velodyne_camera = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), "velodyne", "stereo");
  if (publish_tf_) br.sendTransform(tf_velodyne_camera);

  laserReceived = false;
  laser_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  ilaser_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  cameraReceived = false;
  camera_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  icamera_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  ros::Subscriber laser_sub = nh_.subscribe<velo2cam_calibration::ClusterCentroids>("cloud1", 1, laser_callback);
  ros::Subscriber stereo_sub = nh_.subscribe<velo2cam_calibration::ClusterCentroids>("cloud2", 1, stereo_callback);

  t_pub = nh_.advertise<sensor_msgs::PointCloud2>("translated_cloud", 1);
  clusters_c = nh_.advertise<sensor_msgs::PointCloud2>("clusters_camera", 1);
  clusters_l = nh_.advertise<sensor_msgs::PointCloud2>("clusters_laser", 1);

  if (save_to_file_){
    ostringstream os;
    os << getenv("HOME") << "/results_" << currentDateTime() << ".csv" ;
    if (save_to_file_){
      if(DEBUG) ROS_INFO("Opening %s", os.str().c_str());
      savefile.open (os.str().c_str());
      savefile << "it, x, y, z, r, p, y, used_l, used_c" << endl;
    }
  }

  ros::Rate loop_rate(30);
  while(ros::ok()){
    ros::spinOnce();
  }

  if (save_to_file_) savefile.close();

  // Save calibration params to launch file for testing

  // Get time
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
  std::string str(buffer);

  // Get tf data
  tf::Transform inverse = tf_velodyne_camera.inverse();
  double roll, pitch, yaw;
  double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(), zt = inverse.getOrigin().getZ();
  inverse.getBasis().getRPY(roll, pitch, yaw);

  ROS_INFO("Calibration finished succesfully...");
  ROS_INFO("x=%.4f y=%.4f z=%.4f",xt,yt,zt);
  ROS_INFO("roll=%.4f, pitch=%.4f, yaw=%.4f", roll, pitch, yaw);

  std::string path = ros::package::getPath("velo2cam_calibration");
  string backuppath = path + "/launch/calibrated_tf_"+ str +".launch";
  path = path + "/launch/calibrated_tf.launch";

  cout << endl << "Creating .launch file with calibrated TF in: "<< endl << path.c_str() << endl;
  // Create .launch file with calibrated TF
  TiXmlDocument doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "utf-8", "");
  doc.LinkEndChild( decl );
  TiXmlElement * root = new TiXmlElement( "launch" );
  doc.LinkEndChild( root );

  TiXmlElement * arg = new TiXmlElement( "arg" );
  arg->SetAttribute("name","stdout");
  arg->SetAttribute("default","screen");
  root->LinkEndChild( arg );

  string stereo_rotation = "0 0 0 -1.57079632679 0 -1.57079632679 stereo stereo_camera 10";

  TiXmlElement * stereo_rotation_node = new TiXmlElement( "node" );
  stereo_rotation_node->SetAttribute("pkg","tf");
  stereo_rotation_node->SetAttribute("type","static_transform_publisher");
  stereo_rotation_node->SetAttribute("name","stereo_ros_tf");
  stereo_rotation_node->SetAttribute("args", stereo_rotation);
  root->LinkEndChild( stereo_rotation_node );

  std::ostringstream sstream;
  sstream << xt << " " << yt << " " << zt << " " << yaw << " " <<pitch<< " " << roll << " stereo velodyne 100";
  string tf_args = sstream.str();
  cout << tf_args << endl;

  TiXmlElement * node = new TiXmlElement( "node" );
  node->SetAttribute("pkg","tf");
  node->SetAttribute("type","static_transform_publisher");
  node->SetAttribute("name","l2c_tf");
  node->SetAttribute("args", tf_args);
  root->LinkEndChild( node );

  // Save XML file and copy
  doc.SaveFile(path);
  doc.SaveFile(backuppath);

  if(DEBUG) cout << "Calibration process finished." << endl;

  return 0;
}
