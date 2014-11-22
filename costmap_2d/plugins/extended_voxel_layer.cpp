#include <costmap_2d/extended_voxel_layer.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/GridCells.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ExtendedVoxelLayer, costmap_2d::Layer)

namespace costmap_2d
{

void ExtendedVoxelLayer::onInitialize()
{
  VoxelLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("combination_method", combination_method_, 1);
  ROS_INFO("%s is using combination_method %d", name_.c_str(), combination_method_);

  std::string clearing_publish_topic;
  private_nh.param("clearing_publish_topic", clearing_publish_topic, std::string());

  if (clearing_publish_topic != std::string())
  {
    update_publisher_ = private_nh.advertise<nav_msgs::GridCells>(clearing_publish_topic, 100);
    ROS_INFO("%s: Publishes clearing updates to topic %s", name_.c_str(), update_publisher_.getTopic().c_str());
  }
}

void ExtendedVoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  VoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ExtendedVoxelLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  if (update_publisher_)
  {
    publishClearing();
  }

  VoxelLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

void ExtendedVoxelLayer::publishClearing()
{
  unsigned int x = 0;
  unsigned int y = 0;

  unsigned char value = 0;
  std::list<geometry_msgs::Point> points;

  for (std::list<MapLocation>::iterator updated_cell_index_it = updated_cells_index_.begin();
      updated_cell_index_it != updated_cells_index_.end(); ++updated_cell_index_it)
  {
    x = (*updated_cell_index_it).x;
    y = (*updated_cell_index_it).y;
    value = costmap_[getIndex(x, y)];

    if (value == FREE_SPACE || value == NO_INFORMATION)
    {
      geometry_msgs::Point point;
      point.x = x;
      point.y = y;
      point.z = FREE_SPACE;
      points.push_back(point);
    }
  }

  nav_msgs::GridCells free_grid_cells;
  free_grid_cells.header.stamp = ros::Time();
  free_grid_cells.cells.resize(points.size());

  ROS_DEBUG("%s is publishing clearing: %d cells", name_.c_str(), (unsigned int )points.size());

  int cell_index = 0;

  for (std::list<geometry_msgs::Point>::iterator points_it = points.begin(); points_it != points.end(); ++points_it)
  {
    free_grid_cells.cells[cell_index] = *points_it;
    cell_index++;
  }

  update_publisher_.publish(free_grid_cells);
}

} //end of namespace costmap_2d
