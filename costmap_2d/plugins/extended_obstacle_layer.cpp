#include <costmap_2d/extended_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ExtendedObstacleLayer, costmap_2d::Layer)

namespace costmap_2d
{

void ExtendedObstacleLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("combination_method", combination_method_, 1);
  ROS_INFO("%s is using combination_method %d", name_.c_str(), combination_method_);

  ros::NodeHandle global_nh;

  std::string update_subscribe_topic;
  private_nh.param("update_subscribe_topic", update_subscribe_topic, std::string());
  if (update_subscribe_topic != std::string())
  {

    std::string name_space = ros::names::parentNamespace(private_nh.getNamespace()).append("/").append(update_subscribe_topic);
    update_subscriber_ = global_nh.subscribe(name_space, 1, &ExtendedObstacleLayer::updateMap, this);
  }
}

void ExtendedObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  resetMaps();
  ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ExtendedObstacleLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

void ExtendedObstacleLayer::updateMap(const nav_msgs::GridCells::Ptr& update_cells)
{
  unsigned int x = 0;
  unsigned int y = 0;

  unsigned char value = 0;
  std::list<geometry_msgs::Point> points;

//  ROS_INFO_STREAM_THROTTLE(3, "updateing " << name_ << " with " << update_cells->cells.size());
  for (int i = 0; i < update_cells->cells.size(); ++i)
  {
    x = update_cells->cells[i].x;
    y = update_cells->cells[i].y;

    costmap_[getIndex(x, y)] = FREE_SPACE;
  }
}

} //end of namespace costmap_2d
