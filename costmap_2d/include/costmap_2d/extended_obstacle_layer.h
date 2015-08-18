#ifndef COSTMAP_2D_EXTENDED_OBSTACLE_LAYER_H_
#define COSTMAP_2D_EXTENDED_OBSTACLE_LAYER_H_

#include <costmap_2d/voxel_layer.h>
#include <nav_msgs/GridCells.h>

namespace costmap_2d
{

class ExtendedObstacleLayer : public ObstacleLayer
{
public:
  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void updateMap(const nav_msgs::GridCells::Ptr& update_cells);

protected:
  boost::shared_ptr<Costmap2D> linked_layer_;
  ros::Subscriber update_subscriber_;

  bool reset_every_cycle_;
};

} //end of namespace costmap_2d

#endif /* COSTMAP_2D_EXTENDED_OBSTACLE_LAYER_H_ */
