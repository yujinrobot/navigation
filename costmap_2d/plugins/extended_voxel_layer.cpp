#include <costmap_2d/extended_voxel_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ExtendedVoxelLayer, costmap_2d::Layer)

namespace costmap_2d
{

void ExtendedVoxelLayer::onInitialize()
{
  VoxelLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("combination_method", combination_method_, 1);
  ROS_INFO("%s is using combination_method %d", name_.c_str(), combination_method_);

  private_nh.param("reset_voxels_every_cycle", reset_voxels_every_cycle_, false);

  std::string clearing_publish_topic;
  private_nh.param("clearing_publish_topic", clearing_publish_topic, std::string());

  if (clearing_publish_topic != std::string())
  {
    update_publisher_ = private_nh.advertise<nav_msgs::GridCells>(clearing_publish_topic, 100);
    ROS_INFO("%s: Publishes clearing updates to topic %s", name_.c_str(), update_publisher_.getTopic().c_str());
  }

  ros::NodeHandle global_nh;

  std::string update_subscribe_topic;
  private_nh.param("update_subscribe_topic", update_subscribe_topic, std::string());
  if (update_subscribe_topic != std::string())
  {

    std::string name_space = ros::names::parentNamespace(private_nh.getNamespace()).append("/").append(
        update_subscribe_topic);
    update_subscriber_ = global_nh.subscribe(name_space, 1, &ExtendedVoxelLayer::updateMap, this);
  }
}

void ExtendedVoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
  if (reset_voxels_every_cycle_)
  {
    resetGrid();
  }

  VoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ExtendedVoxelLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !updated_cells_indices_)
    return;

  if (update_publisher_)
  {
    publishClearing();
  }

  if (combination_method_ <= 1)
  {
    //the old way
    VoxelLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j); //original
    return;
  }

  if (combination_method_ == 2)
  {
    //still add all obstacles
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);

    //selective overwriting
    unsigned char* master_array = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    int x = 0;
    int y = 0;
    int master_index = 0;

    for (std::list<std::pair<unsigned int, unsigned int> >::iterator updated_cell_index_it =
        updated_cells_indices_->begin(); updated_cell_index_it != updated_cells_indices_->end();
        ++updated_cell_index_it)
    {
      x = updated_cell_index_it->first;
      y = updated_cell_index_it->second;

      master_array[y * span + x] = costmap_[getIndex(x, y)];
    }
  }
}

void ExtendedVoxelLayer::updateMap(const nav_msgs::GridCells::Ptr& update_cells)
{
  unsigned int x = 0;
  unsigned int y = 0;

  unsigned char value = 0;
  std::list<geometry_msgs::Point> points;

//  ROS_INFO_STREAM_THROTTLE(3, "updated cells: " << update_cells->cells.size() << ", name: " << name_);

  for (int i = 0; i < update_cells->cells.size(); ++i)
  {
    x = update_cells->cells[i].x;
    y = update_cells->cells[i].y;

    costmap_[getIndex(x, y)] = FREE_SPACE;
  }
}

void ExtendedVoxelLayer::publishClearing()
{
  unsigned int x = 0;
  unsigned int y = 0;

  unsigned char value = 0;
  std::list<geometry_msgs::Point> points;

  for (std::list<std::pair<unsigned int, unsigned int> >::iterator updated_cell_index_it =
      updated_cells_indices_->begin(); updated_cell_index_it != updated_cells_indices_->end(); ++updated_cell_index_it)
  {
    x = updated_cell_index_it->first;
    y = updated_cell_index_it->second;
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

  int cell_index = 0;

  for (std::list<geometry_msgs::Point>::iterator points_it = points.begin(); points_it != points.end(); ++points_it)
  {
    free_grid_cells.cells[cell_index] = *points_it;
    cell_index++;
  }

  update_publisher_.publish(free_grid_cells);
}

} //end of namespace costmap_2d
