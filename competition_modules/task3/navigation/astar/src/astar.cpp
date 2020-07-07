#include "astar.h"

float Astar::distance(Node *a, Node *b)
{
  return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

float Astar::heuristic(Node *a, Node *b)
{
  return distance(a, b) + a->cost;
}

Node Astar::pose2node(Pose *p, OccupancyGrid *map)
{
  Node n;
  n.x = int((p->position.x - map->info.origin.position.x) / map->info.resolution);
  n.y = int((p->position.y - map->info.origin.position.y) / map->info.resolution);
  // cout << n.x << "," << n.y << endl;
  return n;
}

bool Astar::check_valid(Node n, OccupancyGrid map)
{
  if (n.x < 0 || n.x >= map.info.width || n.y < 0 || n.y >= map.info.height)
  {
    return false;
  }
  return true;
}

Astar::Astar()
{
}

bool Astar::initialize(Pose start, Pose end, OccupancyGrid map)
{
  Node ns = pose2node(&start, &map);
  Node ne = pose2node(&end, &map);
  if (check_valid(ns, map) && check_valid(ne, map))
  {
    if (ns.x == ne.x && ns.y == ne.y)
    {
      ROS_WARN("start and goal at the same grid");
      return false;
    }
  }
  else
  {
    ROS_WARN("point outof bound");
    return false;
  }
  this->map = map;
  int nMapWidth = map.info.width;
  int nMapHeight = map.info.height;
  nodes = new Node[nMapWidth * nMapHeight];
  for (int x = 0; x < nMapWidth; x++)
  {
    for (int y = 0; y < nMapHeight; y++)
    {
      int idx = x * nMapHeight + y;

      if (x == ns.x && y == ns.y)
      {
        this->start = &nodes[idx];
      }
      else if (x == ne.x && y == ne.y)
      {
        this->end = &nodes[idx];
      }

      nodes[idx].x = x;
      nodes[idx].y = y;
      nodes[idx].cost = int(map.data[x + y*nMapWidth]) ;
      nodes[idx].is_obstacle = (nodes[idx].cost > 50) ? true : false;
      nodes[idx].parent = nullptr;

      // neighbors
      if (y > 0)
        nodes[idx].neighbors.push_back(&nodes[(y - 1) + (x + 0) * nMapHeight]);
      if (y < nMapHeight - 1)
        nodes[idx].neighbors.push_back(&nodes[(y + 1) + (x + 0) * nMapHeight]);
      if (x > 0)
        nodes[idx].neighbors.push_back(&nodes[(y + 0) + (x - 1) * nMapHeight]);
      if (x < nMapWidth - 1)
        nodes[idx].neighbors.push_back(&nodes[(y + 0) + (x + 1) * nMapHeight]);

      // diagonal neighbors
      if (y > 0 && x > 0)
        nodes[idx].neighbors.push_back(&nodes[(y - 1) + (x - 1) * nMapHeight]);
      if (y < nMapHeight - 1 && x > 0)
        nodes[idx].neighbors.push_back(&nodes[(y + 1) + (x - 1) * nMapHeight]);
      if (y > 0 && x < nMapWidth - 1)
        nodes[idx].neighbors.push_back(&nodes[(y - 1) + (x + 1) * nMapHeight]);
      if (y < nMapHeight - 1 && x < nMapWidth - 1)
        nodes[idx].neighbors.push_back(&nodes[(y + 1) + (x + 1) * nMapHeight]);
    }
  }
  return true;
}

bool Astar::plan(Path *path)
{
  if (nodes == NULL)
  {
    ROS_WARN("please init astar first");
    return false;
  }

  Node *c_node = start;
  start->local_goal = 0;
  start->global_goal = heuristic(start, end);
  list<Node *> open_nodes;
  open_nodes.push_back(start);

  while (!open_nodes.empty() && c_node != end)
  {
    open_nodes.sort([](const Node *lhs, const Node *rhs) { return lhs->global_goal < rhs->global_goal; });

    while (!open_nodes.empty() && open_nodes.front()->visited)
    {
      open_nodes.pop_front();
    }

    if (open_nodes.empty())
    {
      break;
    }

    c_node = open_nodes.front();
    c_node->visited = true;
    // cout << c_node->x << "," << c_node->y << endl;

    for (auto neighbor_node : c_node->neighbors)
    {
      if (!neighbor_node->visited && !neighbor_node->is_obstacle)
      {
        open_nodes.push_back(neighbor_node);
      }

      float new_local_goal = c_node->local_goal + distance(c_node, neighbor_node);

      if (new_local_goal < neighbor_node->local_goal)
      {
        neighbor_node->parent = c_node;
        neighbor_node->local_goal = new_local_goal;
        neighbor_node->global_goal = neighbor_node->local_goal + heuristic(neighbor_node, end);
      }
    }
  }
  if (end->parent == NULL)
  {
    return false;
  }

  path->poses.clear();
  c_node = end;
  while (c_node->parent != NULL)
  {
    // cout << c_node->x << "," << c_node->y << endl;
    PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = Time::now();
    p.pose.position.x = c_node->x * map.info.resolution + map.info.origin.position.x;
    p.pose.position.y = c_node->y * map.info.resolution + map.info.origin.position.y;
    p.pose.position.z = 0;
    p.pose.orientation.w = 1;
    path->poses.push_back(p);

    c_node = c_node->parent;
  }
  // cout << "path ready" << endl;

  reverse(path->poses.begin(), path->poses.end());

  // cout << "path reverse" << endl;

  delete[] nodes;
  return true;
}

Astar::~Astar()
{
}