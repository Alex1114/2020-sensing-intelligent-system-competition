#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <pose_estimation/get_pick_pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include "conversion.hpp"

/*
 * Parameters:
 *   [int] min_point_num*
 *   [int] max_point_num*
 *   [int] buzzer_point_bound*
 *   [int] mid_point_bound*
 *   [double] z_lower*
 *   [double] leaf_size*
 *   [string] arm_prefix
 * [Note] parameter with `*` can be changed in runtime
 */

/*
 * XXX
 * [Note] Minimum Euclidean distance to split objects: > 1.5 cm
 */

pcl::PointCloud<pcl::PointXYZ> segment_plane(pcl::PointCloud<pcl::PointXYZ> pc_in){
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.001f);
  seg.setInputCloud(pc_in.makeShared());
  seg.segment(*inliers, *coefficients);
  pcl::PointCloud<pcl::PointXYZ> plane_cloud(pc_in, inliers->indices);
  return plane_cloud;
}

class ObjClusters{
 private:
  int min_point_num, max_point_num; // Parameters for Euclidean cluster
  int buzzer_point_bound, mid_point_bound; // Parameter to distinguish buzzer, midium transformer and big transformer, according to the point number
  double z_lower, // Only consider points where z belongs to [z_lower, Z_UPPER] 
         leaf_size, // Leaf size for voxel grid downsampling
         min_cluster_dis;  // Parameters for Euclidean cluster
  const double Z_UPPER = 0.2;
  std::string arm_prefix, // Prefix of robot arm 
              pc_frame; // Which frame the pointcloud respect to
  ros::NodeHandle nh_, pnh_; // ROS node handler
  ros::Publisher pub_clusters, pub_plane_cloud;
  ros::Subscriber sub_pc; 
  ros::ServiceServer get_clusters_srv;
  ros::Timer check_parameter_timer; // Timer to check if parameters changed
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_ptr; // Pointer to save subscribed pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_large_cuboid, model_medium_cuboid, model_cylinder;
  pcl::VoxelGrid<pcl::PointXYZRGB> vg; 
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  /*
   * Get transform from target frame to source frame
   * [in] target: target frame name
   * [in] source: source frame name
   * [out] result: if we successfully get transformation
   */
  tf::Transform getTransform(std::string target, std::string source, bool &result){
    tf::StampedTransform stf;
    static tf::TransformListener listener;
    try{
      listener.waitForTransform(target, source, ros::Time(0), ros::Duration(0.5));
      listener.lookupTransform(target, source, ros::Time(0), stf); result = true;
    } catch(tf::TransformException &ex){
      ROS_WARN("[%s] Can't get transform from [%s] to [%s]", \
                ros::this_node::getName().c_str(),
                target.c_str(),
                source.c_str()); result = false;
    }
    return (tf::Transform(stf.getRotation(), stf.getOrigin()));
  }
  // Timer callback, to check if some runtime changable parameters are changed
  void timer_cb(const ros::TimerEvent& event){
    int i_tmp;
    pnh_.getParam("min_point_num", i_tmp);
    if(i_tmp!=min_point_num){
      ROS_INFO("[%s] min_point_num set from %d to %d", ros::this_node::getName().c_str(), min_point_num, i_tmp);
      min_point_num = i_tmp;
    }
    pnh_.getParam("max_point_num", i_tmp);
    if(i_tmp!=max_point_num){
      ROS_INFO("[%s] max_point_num set from %d to %d", ros::this_node::getName().c_str(), max_point_num, i_tmp);
      max_point_num = i_tmp;
    }
    pnh_.getParam("buzzer_point_bound", i_tmp);
    if(i_tmp!=buzzer_point_bound){
      ROS_INFO("[%s] buzzer_point_bound set from %d to %d", ros::this_node::getName().c_str(), buzzer_point_bound, i_tmp);
      buzzer_point_bound = i_tmp;
    }
    pnh_.getParam("mid_point_bound", i_tmp);
    if(i_tmp!=mid_point_bound){
      ROS_INFO("[%s] mid_point_bound set from %d to %d", ros::this_node::getName().c_str(), mid_point_bound, i_tmp);
      mid_point_bound = i_tmp;
    }
    double d_tmp;
    pnh_.getParam("z_lower", d_tmp);
    if(d_tmp!=z_lower){
      ROS_INFO("[%s] z_lower set from %f to %f", ros::this_node::getName().c_str(), z_lower, d_tmp);
      z_lower = d_tmp;
    }
    pnh_.getParam("leaf_size", d_tmp);
    if(d_tmp!=leaf_size){
      ROS_INFO("[%s] leaf_size set from %f to %f", ros::this_node::getName().c_str(), leaf_size, d_tmp);
      leaf_size = d_tmp;
      vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    }
    /*
    pnh_.getParam("min_cluster_dis", d_tmp);
    if(d_tmp!=min_cluster_dis){
      ROS_INFO("[%s] min_cluster_dis set from %f to %f", ros::this_node::getName().c_str(), min_cluster_dis, d_tmp);
      min_cluster_dis = d_tmp;
    }
    */
  }
  // Subscriber callback, only update pointcloud pointer
  void sub_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    if(pc_frame.empty()) pc_frame = msg->header.frame_id;
    pcl::fromROSMsg(*msg, *pc_ptr);
  }
  /*
   * Segmentation
   * Using spatial prior knownledge to segment where are our targets
   * 1. Remove NaN points from sensor data
   * 2. Change pointcloud coordinate from eye to hand with transformation
   * 3. Remove outliers that don't belong to [z_lower, Z_UPPER]
   * 4. Remove statistical outliers
   * 5. Run Euclidean cluster
   * 6. Return the biggest cluster pointcloud
   * [out] cluster_num: number of clusters
   * return: cluster pointcloud with most points
   */
  
  pcl::PointCloud<pcl::PointXYZRGB> segmentation(int &cluster_num){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed (new pcl::PointCloud<pcl::PointXYZRGB>()), 
                                           pub_pc (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // Remove NaN points
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*pc_ptr, *processed, mapping);
    #ifdef VERBOSE
      std::cout << "Remove nan: " << processed->points.size() << "\n";
    #endif
    // Change frame
    bool can_get_transform;
    Eigen::Matrix4f transform_mat = tf2eigen(getTransform(arm_prefix+"base_link", pc_frame, can_get_transform));
    if(!can_get_transform){
      return *pub_pc; // Empty cloud
    }
    pcl::transformPointCloud(*processed, *processed, transform_mat);
    // Conditional removal
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, z_lower)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, Z_UPPER)));
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(processed);
    condrem.filter(*processed);
    #ifdef VERBOSE
      std::cout << "Conditional removal: " << processed->points.size() << "\n";
    #endif
    if(processed->points.size()==0) return *pub_pc; // Empty cloud
    // Downsampling
    vg.setInputCloud(processed);
    vg.filter(*processed);
    #ifdef VERBOSE
      std::cout << "Downsampling: " << processed->points.size() << "\n";
    #endif
    // Statistical outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(processed);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*processed);
    #ifdef VERBOSE
      std::cout << "Statistical outlier removal: " << processed->points.size() << "\n";
    #endif
    // Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(processed);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(min_cluster_dis);
    ec.setMinClusterSize(min_point_num);
    ec.setMaxClusterSize(max_point_num);
    ec.setSearchMethod(tree);
    ec.setInputCloud(processed);
    ec.extract(cluster_indices);
    std::stringstream ss;
    for(int i=0; i<cluster_indices.size(); ++i){ // In points number order
      pcl::PointCloud<pcl::PointXYZRGB> cluster_pc(*processed, cluster_indices[i].indices);
      *pub_pc += cluster_pc;
      ss << "Cluster " << i << " with " << cluster_indices[i].indices.size() << " points\n";
    } cluster_num = cluster_indices.size();
    ROS_INFO("[%s] Detect %d clusters.", ros::this_node::getName().c_str(), (int)cluster_indices.size()); 
    #ifdef VERBOSE
    std::cout << ss.str();
    #endif
    if(cluster_num!=0){
      sensor_msgs::PointCloud2 pc_out;
      pcl::toROSMsg(*pub_pc, pc_out);
      pc_out.header.frame_id = arm_prefix+"base_link";
      pub_clusters.publish(pc_out);
      pcl::PointCloud<pcl::PointXYZRGB> most_important_pc(*processed, cluster_indices[0].indices);
      return most_important_pc;
    } else return *pub_pc; // Empty
  }
  /*
   * Service callback
   */
  bool srv_cb(pose_estimation::get_pick_pose::Request &req, pose_estimation::get_pick_pose::Response &res){
    ROS_INFO("[%s] Receive new request, start processing...", ros::this_node::getName().c_str());
    if(pc_ptr->points.size()==0){
      ROS_ERROR("[%s] No points received yet, make sure you connect the topic correctly!", ros::this_node::getName().c_str());
      res.cluster_num = 0; res.cluster_points = 0; res.status = "no_cloud";
      return true;
    }
    int cluster_num;
    auto obj_pc = segmentation(cluster_num);
    res.cluster_num = cluster_num;
    if(cluster_num==0){
      ROS_ERROR("[%s] No cluster detected, abort request...", ros::this_node::getName().c_str());
      res.status = "no_cluster_detected";
      res.cluster_points = 0;
      return true;
    }
    pcl::PointCloud<pcl::PointXYZ> obj_pc_xyz;
    // Can't use copyPointCloud from xyzrgb to xyz, so using for to copy the pointcloud
    for(auto p_: obj_pc.points){
      pcl::PointXYZ p;
      p.x = p_.x;
      p.y = p_.y;
      p.z = p_.z;
      obj_pc_xyz.points.push_back(p);
    } obj_pc_xyz.width = 1; obj_pc_xyz.height = obj_pc.points.size();
    res.cluster_points = obj_pc.points.size();
    if(obj_pc.points.size()==0){
      ROS_ERROR("[%s] Can't get transform or empty cloud, abort request...", ros::this_node::getName().c_str());
      res.status = "no_cloud";
      res.cluster_points = 0;
      return true;
    }
    tf::Transform final_tf;
    /* Method: PCA
    if(res.cluster_points>buzzer_point_bound){
      res.obj_str = (res.cluster_points>mid_point_bound?"large_cuboid":"medium_cuboid");
      res.obj_id  = (res.cluster_points>mid_point_bound?0:2);
      // Plane segmentation
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.001f);
      seg.setInputCloud(obj_pc_xyz.makeShared());
      seg.segment(*inliers, *coefficients);
      pcl::PointCloud<pcl::PointXYZ> plane_cloud(obj_pc_xyz, inliers->indices);
      sensor_msgs::PointCloud2 pc_out;
      pcl::toROSMsg(plane_cloud, pc_out);
      pc_out.header.frame_id = arm_prefix+"base_link";
      pub_plane_cloud.publish(pc_out);
      // PCA to obtain object frame
      pcl::PCA<pcl::PointXYZ> pca;
      pca.setInputCloud(plane_cloud.makeShared());
      Eigen::Matrix3f ev = pca.getEigenVectors(); Eigen::Vector4f centroid = pca.getMean();
      Eigen::Matrix4f transform_eigen; transform_eigen.block<3, 3>(0, 0) = ev; transform_eigen.block<4, 1>(0, 3) = centroid;
      final_tf = eigen2tf_full(transform_eigen);
      if(final_tf.getBasis().getColumn(2).getZ()<0.0f) // Force Z-axis face upward (i.e., +Z of base_link)
        // Rotate X about 180 degree
        final_tf *= tf::Transform(tf::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), tf::Vector3(0.0f, 0.0f, 0.0f));
      // Convert object frame to picking frame, [0, -1, 0; 0, 0, 1; -1, 0, 0]
      final_tf *= tf::Transform(tf::Quaternion(0.5f, -0.5f, -0.5f, -0.5f), tf::Vector3(0.0f, 0.0f, 0.0f));
      if(final_tf.getBasis().getColumn(2).getX()>0.0f) // Wrist 3 can't rotate too much or the camera will be collided
        // Rotate X about 180 degree
        final_tf *= tf::Transform(tf::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), tf::Vector3());
      // Planar grasping
      double theta = acos(final_tf.getBasis().getColumn(0).dot(tf::Vector3(0.0f, 0.0f, -1.0f)));
      tf::Vector3 line_vec = final_tf.getBasis().getColumn(0).cross(tf::Vector3(0.0, 0.0f, -1.0f));
      line_vec = line_vec.normalized();
      tf::Quaternion q(line_vec, theta), q_1(line_vec, -theta);
      tf::Transform t(q, tf::Vector3(0.0f, 0.0f, 0.0f)), t_1(q_1, tf::Vector3(0.0f, 0.0f, 0.0f)), tmp = t*final_tf, tmp_1 = t_1*final_tf;
      if(fabs(tmp.getBasis().getColumn(0).getZ())>fabs(tmp_1.getBasis().getColumn(0).getZ())) final_tf = tmp;
      else final_tf = tmp_1;
      res.status = "success";
    } else{ // Cluster with few points, considered as buzzer
      ROS_INFO("[%s] Cluster with too few points, viewed as buzzer", ros::this_node::getName().c_str());
      res.obj_id = 1;
      res.obj_str = "cylinder";
      res.status = "success";
      // Plane segmentation
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.001f);
      seg.setInputCloud(obj_pc_xyz.makeShared());
      seg.segment(*inliers, *coefficients);
      pcl::PointCloud<pcl::PointXYZ> plane_cloud(obj_pc_xyz, inliers->indices);
      sensor_msgs::PointCloud2 pc_out;
      pcl::toROSMsg(plane_cloud, pc_out);
      pc_out.header.frame_id = arm_prefix+"base_link";
      pub_plane_cloud.publish(pc_out);
      pcl::CentroidPoint<pcl::PointXYZ> cp;
      for(auto p: plane_cloud.points) cp.add(p);
      pcl::PointXYZ centroid; cp.get(centroid);
      // Simply give [0, 0, -1; 0, -1, 0; -1, 0, 0] for cylinder object since it is all-axis symmetric
      final_tf = tf::Transform(tf::Quaternion(0.707f, 0.0f, -0.707f, 0.0f), tf::Vector3(centroid.x, centroid.y, centroid.z));
    } */
    // Method: ICP
    icp.setInputSource(obj_pc_xyz.makeShared());
    if(res.cluster_points>buzzer_point_bound){
      res.obj_str = (res.cluster_points>mid_point_bound?"large_cuboid":"medium_cuboid");
      res.obj_id  = (res.cluster_points>mid_point_bound?0:2);
      if(res.obj_str=="large_cuboid"){ // Large cuboid
        icp.setInputTarget(model_large_cuboid);
      }else // Medium cuboid
        icp.setInputTarget(model_medium_cuboid);
      Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
      initial_guess(0, 3) = 0.4f;
      pcl::PointCloud<pcl::PointXYZ> icp_res_pc;
      icp.align(icp_res_pc, initial_guess);
      Eigen::Matrix4f res_mat = icp.getFinalTransformation();
      final_tf = eigen2tf_full(res_mat).inverse();
      // Convert to picking frame, [0, 0, -1; 0, -1, 0; -1, 0, 0]
      final_tf *= tf::Transform(tf::Quaternion(-0.707f, 0.0f, 0.707f, 0.0f), tf::Vector3(0.0f, 0.0f, 0.0f));
      // Planar grasping, X axis should face downward (i.e., (0, 0, -1) in base_link)
      double theta = acos(final_tf.getBasis().getColumn(0).dot(tf::Vector3(0.0f, 0.0f, -1.0f)));
      tf::Vector3 line_vec = final_tf.getBasis().getColumn(0).cross(tf::Vector3(0.0, 0.0f, -1.0f));
      line_vec = line_vec.normalized();
      tf::Quaternion q(line_vec, theta), q_1(line_vec, -theta);
      tf::Transform t(q, tf::Vector3(0.0f, 0.0f, 0.0f)), t_1(q_1, tf::Vector3(0.0f, 0.0f, 0.0f)), tmp = t*final_tf, tmp_1 = t_1*final_tf;
      if(fabs(tmp.getBasis().getColumn(0).getZ())>fabs(tmp_1.getBasis().getColumn(0).getZ())) final_tf = tmp;
      else final_tf = tmp_1;
      auto plane_cloud = segment_plane(obj_pc_xyz);
      // Dominant vector should along Z axis
      /*
      pcl::PCA<pcl::PointXYZ> pca;
      pca.setInputCloud(plane_cloud.makeShared());
      Eigen::Matrix3f ev = pca.getEigenVectors(); Eigen::Vector4f c = pca.getMean();
      tf::Vector3 dominant_vector(ev(0, 0), ev(1, 0), ev(2, 0)); dominant_vector = dominant_vector.normalized();
      Eigen::Matrix4f eigen_mat; eigen_mat.block<3, 3>(0, 0) = ev; eigen_mat.block<4, 1>(0, 3) = c;
      tf::Transform eigen = eigen2tf_full(eigen_mat);
      ros::Time now = ros::Time::now();
      while((ros::Time::now()-now).toSec()<0.2){
        static tf::TransformBroadcaster br; br.sendTransform(tf::StampedTransform(eigen, ros::Time::now(), arm_prefix+"base_link", "eigen"));
      }
      std::cout << "Dot product: " << dominant_vector.dot(final_tf.getBasis().getColumn(2)) << "\n";
      if(dominant_vector.dot(final_tf.getBasis().getColumn(2))<0.173f){ // ~ 80 degree
        // Rotate X axis about 90 degree
        final_tf *= tf::Transform(tf::Quaternion(0.707f, 0.0f, 0.0f, 0.707f), tf::Vector3(0.0, 0.0f, 0.0f));
        std::cout << "rotate\n";
      }
      */
      // Prevent self-collision since wrist 3 is restricted by camera
      if(final_tf.getBasis().getColumn(2).getX()<0.0f)
        // Rotate X axis about 180 degree
        final_tf *= tf::Transform(tf::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), tf::Vector3(0.0f, 0.0f, 0.0f)); 
    }else{ // Cylinder
      res.obj_str = "cylinder";
      res.obj_id  = 1;
      // Simply give [0, 0, -1; 0, -1, 0; -1, 0, 0] for cylinder object since it is all-axis symmetric
      final_tf = tf::Transform(tf::Quaternion(0.707f, 0.0f, -0.707f, 0.0f), tf::Vector3(0.0f, 0.0f, 0.0f));
    }
    // Put origin at the centroid of planar cloud
    auto plane_cloud = segment_plane(obj_pc_xyz);
    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(plane_cloud, pc_out);
    pc_out.header.frame_id = arm_prefix+"base_link";
    pub_plane_cloud.publish(pc_out);
    pcl::CentroidPoint<pcl::PointXYZ> cp;
    for(auto p: plane_cloud.points) cp.add(p);
    pcl::PointXYZ centroid; cp.get(centroid);
    final_tf.setOrigin(tf::Vector3(centroid.x, centroid.y, centroid.z));
    //final_tf *= tf::Transform(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f), tf::Vector3(0.01f, 0.0f, 0.0f)); // Down 1 cm
    res.pick_pose.header.frame_id = "base_link";
    res.pick_pose.header.stamp = ros::Time::now();
    res.pick_pose.pose = tf2Pose(final_tf);
    // Broadcast transformation 0.2 second
    ros::Time ts = ros::Time::now();
    while((ros::Time::now()-ts).toSec()<0.2f){
      static tf::TransformBroadcaster br;
      br.sendTransform(tf::StampedTransform(final_tf, ros::Time::now(), arm_prefix+"base_link", res.obj_str.c_str()));
    }
    res.status = "success";
    ROS_INFO("[%s] Service return", ros::this_node::getName().c_str()); ros::spinOnce();
    return true;
  }
  // Load models and save into pointcloud pointer
  void loadModels(){
    std::string file_directory = ros::package::getPath("pose_estimation")+"/pcd/";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_directory+"large_cuboid.pcd", *model_large_cuboid)==-1 or
       pcl::io::loadPCDFile<pcl::PointXYZ>(file_directory+"medium_cuboid.pcd", *model_medium_cuboid)==-1 or
       pcl::io::loadPCDFile<pcl::PointXYZ>(file_directory+"cylinder.pcd", *model_cylinder)==-1){
      ROS_ERROR("Can't load model, make sure you have put model pcd into \033[1;33mpose_estimation/pcd/\033[0m, shuting down...");
      ros::shutdown();
    }
    // Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vg_model;
    vg_model.setLeafSize(0.003f, 0.003f, 0.003f);
    std::stringstream ss;
    ss << "Loaded model\nlarge_cuboid: " << model_large_cuboid->points.size() << " -> ";
    vg_model.setInputCloud(model_large_cuboid);
    vg_model.filter(*model_large_cuboid);
    ss << model_large_cuboid->points.size() << "\n\
medium_cuboid: " << model_medium_cuboid->points.size() << " -> ";
    vg_model.setInputCloud(model_medium_cuboid);
    vg_model.filter(*model_medium_cuboid);
    ss << model_medium_cuboid->points.size() << "\n\
cylinder: " << model_cylinder->points.size() << " -> ";
    vg_model.setInputCloud(model_cylinder);
    vg_model.filter(*model_cylinder);
    ss << model_cylinder->points.size() << "\n";
    ROS_INFO("%s", ss.str().c_str());
  }
 public:
  // Constructor
  ObjClusters(ros::NodeHandle nh, ros::NodeHandle pnh): min_cluster_dis(0.01f), pc_frame(""), nh_(nh), pnh_(pnh){
    pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    model_large_cuboid.reset(new pcl::PointCloud<pcl::PointXYZ>()); 
    model_medium_cuboid.reset(new pcl::PointCloud<pcl::PointXYZ>()) ; 
    model_cylinder.reset(new pcl::PointCloud<pcl::PointXYZ>());
    loadModels();
    // ID: 0: big transformer, 1: buzzer, 2: mid transformer
    // Get parameters
    if(!pnh_.getParam("arm_prefix", arm_prefix)) arm_prefix="";
    /*
    if(!pnh_.getParam("min_cluster_dis", min_cluster_dis)){
      min_cluster_dis = 0.02f;
      ROS_WARN("[%s] min_cluster_dis not set, using default one...", ros::this_node::getName().c_str());
      pnh_.setParam("min_cluster_dis", min_cluster_dis);
    }
    */
    if(!pnh_.getParam("z_lower", z_lower)){
      z_lower = 0.01f;
      pnh_.setParam("z_lower", z_lower);
    }
    if(!pnh_.getParam("leaf_size", leaf_size)){
      leaf_size = 0.05f;
      pnh_.setParam("leaf_size", leaf_size);
    }
    if(!pnh_.getParam("min_point_num", min_point_num)){
      min_point_num = 50;
      pnh_.setParam("min_point_num", min_point_num);
    }
    if(!pnh_.getParam("max_point_num", max_point_num)){
      max_point_num = 500;
      pnh_.setParam("max_point_num", max_point_num);
    }
    if(!pnh_.getParam("buzzer_point_bound", buzzer_point_bound)){
      buzzer_point_bound = 100;
      pnh_.setParam("buzzer_point_bound", buzzer_point_bound);
    }
    if(!pnh_.getParam("mid_point_bound", mid_point_bound)){
      mid_point_bound = 400;
      pnh_.setParam("mid_point_bound", mid_point_bound);
    }
    // Show parameters
    ROS_INFO("\
[%s]\n============================================\n\
min_point_num: %d\n\
max_point_num: %d\n\
mid_point_bound: %d\n\
buzzer_point_bound: %d\n\
arm_prefix: %s\n\
z_lower: %f\n\
leaf_size: %f\n\
=====================================================", 
ros::this_node::getName().c_str(), 
min_point_num, 
max_point_num, 
mid_point_bound,
buzzer_point_bound,
arm_prefix.c_str(), 
z_lower, 
leaf_size);
    #ifdef VERBOSE
      ROS_WARN("[%s] VERBOSE mode", ros::this_node::getName().c_str());
    #endif
    if(!arm_prefix.empty()) arm_prefix+="_";
    // Downsampling
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    // ICP
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    // Set timer, publisher, subscriber and service
    check_parameter_timer = pnh_.createTimer(ros::Duration(1.0), &ObjClusters::timer_cb, this);
    pub_clusters = pnh_.advertise<sensor_msgs::PointCloud2>("clusters", 1);
    pub_plane_cloud  = pnh_.advertise<sensor_msgs::PointCloud2>("plane_res", 1);
    sub_pc = pnh_.subscribe("pc", 1, &ObjClusters::sub_cb, this);
    get_clusters_srv = pnh_.advertiseService("get_pose", &ObjClusters::srv_cb, this);
  }
  ~ObjClusters(){
    std::string package_path = ros::package::getPath("pose_estimation");
    std::string directory_path = package_path+"/config";
    std::string name_space = ros::this_node::getName();
    boost::filesystem::path p(directory_path);
    if(!boost::filesystem::exists(p)) boost::filesystem::create_directories(p);
    std::string yaml_path = directory_path+"/pose_estimation.yaml";
    std::string command_str = "rosparam dump " + yaml_path + " " + name_space;
    system(command_str.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_pick_pose");
  ros::NodeHandle nh, pnh("~");
  ObjClusters foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}