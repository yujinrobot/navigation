#ifndef COSTMAP_2D_EXTENDED_VOXEL_LAYER_H_
#define COSTMAP_2D_EXTENDED_VOXEL_LAYER_H_

#include <costmap_2d/voxel_layer.h>

namespace costmap_2d
{

class ExtendedVoxelLayer : public VoxelLayer
{
public:
  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  virtual void publishClearing();

  boost::shared_ptr<Costmap2D> linked_layer_;
  ros::Publisher update_publisher_;
};

} //end of namespace costmap_2d

#endif /* DIFFERENTIAL_UPDATE_H_ */
