#include <cmath>
#include <iostream>
#include <vector>

#include "astar.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

using namespace std;
using namespace ros;

class PathPlan
{
private:
  Subscriber sub_click;
  Subscriber sub_map;
  Publisher pub_path;
  ServiceServer plan_service;
  Astar planner;

  nav_msgs::OccupancyGrid::Ptr map;

  void cb_map(const nav_msgs::OccupancyGridPtr &map);
  bool cb_service(nav_msgs::GetPlanRequest &req, nav_msgs::GetPlanResponse &res);

public:
  PathPlan(NodeHandle nh);
  ~PathPlan();
};

PathPlan::PathPlan(NodeHandle nh)
{
  pub_path = nh.advertise<nav_msgs::Path>("astar_path", 1);
  sub_map = nh.subscribe("map", 1, &PathPlan::cb_map, this);
  plan_service = nh.advertiseService("plan_service", &PathPlan::cb_service, this);
}

bool PathPlan::cb_service(nav_msgs::GetPlanRequest &req, nav_msgs::GetPlanResponse &res)
{
  if (this->map == NULL)
  {
    ROS_WARN("map empty");
    return false;
  }
  nav_msgs::Path path;
  if (!planner.initialize(req.start.pose, req.goal.pose, *this->map))
  {
    ROS_WARN("planner init fail");
    return false;
  }

  if (!planner.plan(&path))
  {
    ROS_WARN("planner plan fail");
    return false;
  }

  path.header.stamp = Time::now();
  path.header.frame_id = "global";
  pub_path.publish(path);

  res.plan = path;

  return true;
}

void PathPlan::cb_map(const nav_msgs::OccupancyGridPtr &map)
{
  this->map = map;
}

PathPlan::~PathPlan()
{
}

int main(int argc, char *argv[])
{
  init(argc, argv, "path_plan");
  ROS_INFO("astar path plan init");
  NodeHandle nh;
  PathPlan pathPlan(nh);
  spin();
  return 0;
}
