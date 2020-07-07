#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>


inline geometry_msgs::Pose tf2Pose(const tf::Transform t){
  geometry_msgs::Pose res;
  res.orientation.x = t.getRotation().getX();
  res.orientation.y = t.getRotation().getY();
  res.orientation.z = t.getRotation().getZ();
  res.orientation.w = t.getRotation().getW();
  res.position.x    = t.getOrigin().getX();
  res.position.y    = t.getOrigin().getY();
  res.position.z    = t.getOrigin().getZ();
  return res;
}

inline Eigen::Matrix4f tf2eigen(const tf::Transform t){
  Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
  tf::Matrix3x3 rot_mat(t.getRotation());
  res(0, 0) = rot_mat[0][0]; res(0, 1) = rot_mat[0][1]; res(0, 2) = rot_mat[0][2]; 
  res(1, 0) = rot_mat[1][0]; res(1, 1) = rot_mat[1][1]; res(1, 2) = rot_mat[1][2]; 
  res(2, 0) = rot_mat[2][0]; res(2, 1) = rot_mat[2][1]; res(2, 2) = rot_mat[2][2];
  res(0, 3) = t.getOrigin().getX();
  res(1, 3) = t.getOrigin().getY(); 
  res(2, 3) = t.getOrigin().getZ(); 
  res(3, 3) = 1.0f;
  return res;
}

inline tf::Matrix3x3 eigen2tf(const Eigen::Matrix4f t){
  tf::Matrix3x3 res;
  res[0][0] = t(0, 0); res[0][1] = t(0, 1); res[0][2] = t(0, 2);
  res[1][0] = t(1, 0); res[1][1] = t(1, 1); res[1][2] = t(1, 2);
  res[2][0] = t(2, 0); res[2][1] = t(2, 1); res[2][2] = t(2, 2);
  return res;
}

inline tf::Transform eigen2tf_full(const Eigen::Matrix4f t){
  tf::Matrix3x3 rot_mat = eigen2tf(t);
  tf::Quaternion quat; rot_mat.getRotation(quat);
  tf::Transform res(quat.normalized(), tf::Vector3(t(0, 3), t(1, 3), t(2, 3)));
  return res;
}