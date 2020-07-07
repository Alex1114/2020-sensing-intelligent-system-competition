#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "object_detection/get_mask.h"
#include "object_detection/get_object_pose.h"

using namespace std;
using namespace ros;
using namespace pcl;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudXYZRGBNormal;

class Object_pose_server
{
  public:
    geometry_msgs::PoseStamped pose_3d;
    ros::Publisher centroid_publisher, centroid_marker_publisher, bonding_box_publisher, cloudnum_publisher, pub_cluster;
    ros::ServiceServer service; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_get;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_all;
    tf::TransformListener listener;

    NodeHandle nh;
  
    Object_pose_server(NodeHandle nh);
    visualization_msgs::Marker add_centroid(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, int id);
    visualization_msgs::Marker boxing(PointCloudXYZRGB::Ptr cloud, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id);
    visualization_msgs::Marker add_cloudnum(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id);
    geometry_msgs::PoseArray point_preprocess(PointCloudXYZRGB::Ptr cloud);
    bool pose_request(object_detection::get_object_pose::Request &req, object_detection::get_object_pose::Response &res);
};

Object_pose_server::Object_pose_server(NodeHandle nh){
  this->nh = nh;
  cloud_get.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  cluster_all.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  centroid_publisher = nh.advertise<geometry_msgs::PoseArray>("/centroid_cloud", 1);
  centroid_marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/Marker_centroid", 1);
  bonding_box_publisher = nh.advertise<visualization_msgs::MarkerArray>("/Marker_bonding_box", 1);
  cloudnum_publisher = nh.advertise<visualization_msgs::MarkerArray>("/Marker_cloudnum", 1);
  pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/clustered_prediction",1);

  service = nh.advertiseService("/get_object_pose", &Object_pose_server::pose_request, this); 

}


visualization_msgs::Marker Object_pose_server::add_centroid(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, int id){
  visualization_msgs::Marker cen_pose;
  cen_pose.header.frame_id = "camera_color_optical_frame";
  cen_pose.ns = "basic_shapes";
  cen_pose.id = id;
  cen_pose.type = visualization_msgs::Marker::SPHERE;
  cen_pose.action = visualization_msgs::Marker::ADD;
  cen_pose.lifetime = ros::Duration(2);

  cen_pose.scale.x = 0.007;
  cen_pose.scale.y = 0.007;
  cen_pose.scale.z = 0.007;
  cen_pose.color.r = 1;
  cen_pose.color.g = 0;
  cen_pose.color.b = 0;
  cen_pose.color.a = 1.0;
  cen_pose.pose.position.x = centroid[0];
  cen_pose.pose.position.y = centroid[1];
  cen_pose.pose.position.z = centroid[2];
  cen_pose.pose.orientation.x = 0.0;
  cen_pose.pose.orientation.y = 0.0;
  cen_pose.pose.orientation.z = 0.0;
  cen_pose.pose.orientation.w = 1.0;

  return cen_pose;
}

visualization_msgs::Marker Object_pose_server::boxing(PointCloudXYZRGB::Ptr cloud, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id){

  //實例化一個Momentof...
  pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute ();
//聲明一些必要的變量
  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZRGB min_point_AABB;
  pcl::PointXYZRGB max_point_AABB;
  pcl::PointXYZRGB min_point_OBB;
  pcl::PointXYZRGB max_point_OBB;
  pcl::PointXYZRGB position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
//計算描述符和其他的特徵
  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);
  Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat(rotational_matrix_OBB);
  // cout << "--------------------------------------------------------------" << endl;
  // // cout << "centroid: \n" << centroid << endl;
  // cout << "position: \n" << position << endl;
  // // cout << "rotational: \n" << rotational_matrix_OBB << endl;
  // cout << "quat: \n" << quat.coeffs() << endl;
  pose_3d.pose.position.x = position[0]; pose_3d.pose.position.y = position[1]; pose_3d.pose.position.z = position[2];
  pose_3d.pose.orientation.x = quat.x(); pose_3d.pose.orientation.y = quat.y(); pose_3d.pose.orientation.z = quat.z(); pose_3d.pose.orientation.w = quat.w();
 
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "camera_color_optical_frame";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "box_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.lifetime = ros::Duration(2);

  line_list.id = id;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.002f;
  line_list.color.g = 0.9f;
  line_list.color.b = 0.9f;
  line_list.color.r = 0.9f;
  line_list.color.a = 1.0f;

  Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

  p1 = rotational_matrix_OBB * p1 + position;
  p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  p6 = rotational_matrix_OBB * p6 + position;
  p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  geometry_msgs::Point pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  pt1.x = p1[0]; pt1.y = p1[1]; pt1.z = p1[2]; 
  pt2.x = p2[0]; pt2.y = p2[1]; pt2.z = p2[2]; 
  pt3.x = p3[0]; pt3.y = p3[1]; pt3.z = p3[2]; 
  pt4.x = p4[0]; pt4.y = p4[1]; pt4.z = p4[2]; 
  pt5.x = p5[0]; pt5.y = p5[1]; pt5.z = p5[2]; 
  pt6.x = p6[0]; pt6.y = p6[1]; pt6.z = p6[2]; 
  pt7.x = p7[0]; pt7.y = p7[1]; pt7.z = p7[2]; 
  pt8.x = p8[0]; pt8.y = p8[1]; pt8.z = p8[2]; 

  line_list.points.push_back(pt1);
  line_list.points.push_back(pt2);
  line_list.points.push_back(pt1);
  line_list.points.push_back(pt4);
  line_list.points.push_back(pt1);
  line_list.points.push_back(pt5);

  line_list.points.push_back(pt3);
  line_list.points.push_back(pt2);
  line_list.points.push_back(pt3);
  line_list.points.push_back(pt4);
  line_list.points.push_back(pt3);
  line_list.points.push_back(pt7);

  line_list.points.push_back(pt6);
  line_list.points.push_back(pt2);
  line_list.points.push_back(pt6);
  line_list.points.push_back(pt5);
  line_list.points.push_back(pt6);
  line_list.points.push_back(pt7);

  line_list.points.push_back(pt8);
  line_list.points.push_back(pt4);
  line_list.points.push_back(pt8);
  line_list.points.push_back(pt5);
  line_list.points.push_back(pt8);
  line_list.points.push_back(pt7);

  return line_list;
}

visualization_msgs::Marker Object_pose_server::add_cloudnum(PointCloudXYZRGB::Ptr cloud, Eigen::Vector4f centroid, pcl::PointXYZRGB min, pcl::PointXYZRGB max, int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id="camera_color_optical_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.lifetime = ros::Duration(2);

  marker.scale.z = 0.02f;
  marker.color.b = 1.0f;
  marker.color.g = 1.0f;
  marker.color.r = 1.0f;
  marker.color.a = 1.0f;

  geometry_msgs::Pose pose;
  pose.position.x = centroid[0] - (max.x - min.x);
  pose.position.y = centroid[1] + (max.y - min.y);
  pose.position.z = centroid[2] + (max.z - min.z);
  string str= to_string(cloud->points.size());
  marker.text=str;
  marker.pose=pose;

  return marker;
}

geometry_msgs::PoseArray Object_pose_server::point_preprocess(PointCloudXYZRGB::Ptr cloud)
{
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // =============Cluster=============
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);

  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (3200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  visualization_msgs::MarkerArray cen_poses, bonding_boxes, cloudnum_texts;
  geometry_msgs::PoseArray cens;
  cens.header.stamp = ros::Time::now();
  cens.header.frame_id = "camera_color_optical_frame";

  int currentClusterNum = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator point = it->indices.begin(); point != it->indices.end(); point++)
        cluster->points.push_back(cloud->points[*point]);
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    if (cluster->points.size() <= 0)
      break;

    Eigen::Vector4f centroid; 
    pcl::compute3DCentroid(*cluster, centroid);
    pcl::PointXYZRGB min, max; 
    pcl::getMinMax3D(*cluster, min, max);  

    visualization_msgs::Marker cen_pose = add_centroid(cluster, centroid, currentClusterNum);
    cen_poses.markers.push_back(cen_pose);

    visualization_msgs::Marker bonding_box = boxing(cluster, min, max, currentClusterNum);
    bonding_boxes.markers.push_back(bonding_box);
    cens.poses.push_back(pose_3d.pose);
    
    visualization_msgs::Marker cloudnum = add_cloudnum(cluster, centroid, min, max, currentClusterNum);
    cloudnum_texts.markers.push_back(cloudnum);
    
    currentClusterNum++;
    *cluster_all += *cluster;

  }
  centroid_publisher.publish(cens);
  centroid_marker_publisher.publish(cen_poses);
  bonding_box_publisher.publish(bonding_boxes);
  cloudnum_publisher.publish(cloudnum_texts);


  vector<int> indices2;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices2);

  sensor_msgs::PointCloud2 ros_pc;
  toROSMsg(*cluster_all, ros_pc);
  ros_pc.header.frame_id = "camera_color_optical_frame";
  ros_pc.header.stamp = ros::Time(0);
  pub_cluster.publish(ros_pc);

  cluster_all->clear();
  return cens;
}

bool Object_pose_server::pose_request(object_detection::get_object_pose::Request &req, object_detection::get_object_pose::Response &res){
  ros::ServiceClient client = nh.serviceClient<object_detection::get_mask>("get_maskcloud");
  object_detection::get_mask srv;

  if(client.call(srv)){
    sensor_msgs::PointCloud2 cloud_mask = srv.response.mask_cloud;
    
    fromROSMsg(cloud_mask, *cloud_get);
    
    // tf::StampedTransform tf_c2b;
    // Eigen::Matrix4f c2b;
    // try
    // {
    //     ros::Duration five_seconds(5.0);
    //     listener.waitForTransform("/camera_color_optical_frame", "/base_link", ros::Time(0), five_seconds);
    //     listener.lookupTransform("/camera_color_optical_frame", "/base_link", ros::Time(0), tf_c2b);
    // }
    // catch (tf::TransformException ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    //     return false;
    // }
    // pcl_ros::transformAsMatrix(tf_c2b, c2b);

    // pcl::transformPointCloud(*cloud_get, *cloud_get, c2b);

    geometry_msgs::PoseArray pose_all;
    pose_all = point_preprocess(cloud_get);
    cloud_get->clear();

    
    // find closet one
    float min_z = 100;
    for(int i=0;i<pose_all.poses.size();i++){
      if(pose_all.poses[i].position.z < min_z){
        res.pose = pose_all.poses[i];
        min_z = res.pose.position.z;
      }
    }

    return true;
  }
  else{
    ROS_ERROR("get object pose error");
    return false;
  }
 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_pose");
  ros::NodeHandle nh;

  Object_pose_server server(nh);

  ros::spin();
  return 0;
}
