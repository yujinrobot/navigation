// License: Yujin

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CLEAR_ROTATE_RECOVERY_H_
#define CLEAR_ROTATE_RECOVERY_H_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <boost/shared_ptr.hpp>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>

namespace clear_rotate_recovery
{
/**
 * @class ClearRotateRecovery
 * @brief A recovery behaviour that clears the costmap and then rotates the robot
 */
class ClearRotateRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief Constructor, make sure to call initialise in addition to actually initialise the object
   */
  ClearRotateRecovery();

  /**
   * @brief Initialisation function for the ClearRotateRecovery recovery behaviour
   * @param tf A pointer to a transform listener
   * @param global_costmap A pointer to the global_costmap used by the navigation stack
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name,
                  tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief Run the ClearRotateRecovery recovery behaviour
   *
   * Clears the costmap and then rotates the robot
   */
  void runBehavior();

  /**
   * @brief For retrieving a bumper pressed event
   *
   * @param msg
   */
  void bumperCB(const std_msgs::EmptyConstPtr msg);

private:
  // general
  bool initialised_;
  costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
  costmap_2d::Costmap2D costmap_;
  std::string name_;
  tf::TransformListener* tf_;

  // clear
  double reset_distance_;
  std::set<std::string> clearable_layers_;
  void clear(costmap_2d::Costmap2DROS* costmap);
  void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);

  // rotate
  bool safe_rotation_, bumper_pressed_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
  boost::shared_ptr<base_local_planner::CostmapModel> world_model_;
  ros::Publisher pub_vel_;
  ros::Subscriber sub_bumper_;
  void rotate();
};

} // namespace

#endif /* CLEAR_ROTATE_RECOVERY_H_ */
