#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include <algorithm>
#include <iostream>
#include <list>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

using namespace ros;
using namespace std;
using namespace nav_msgs;
using namespace geometry_msgs;


struct Node
{
  bool is_obstacle = false;
  bool visited = false;
  int cost = 0;
  float global_goal = INFINITY;
  float local_goal = INFINITY;
  int x;
  int y;
  vector<Node *> neighbors;
  Node *parent;
};

class Astar
{
private:
  Node *nodes;
  Node *start;
  Node *end;
  OccupancyGrid map;
  float distance(Node *a, Node *b);
  float heuristic(Node *a, Node *b);
  bool check_valid(Node n, OccupancyGrid map);
  Node pose2node(Pose *p, OccupancyGrid *map);

public:
  Astar();
  bool initialize(Pose start, Pose end, OccupancyGrid map);
  bool plan(Path *path);

  ~Astar();
};

#endif
