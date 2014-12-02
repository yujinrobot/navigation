// License: Yujin

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef HONK_WAIT_RECOVERY_H_
#define HONK_WAIT_RECOVERY_H_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace honk_wait_recovery
{
/**
 * @class HonkWaitRecovery
 * @brief A recovery behaviour that triggers a sound (if supported) and waits for a specified time
 */
class HonkWaitRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief Constructor, make sure to call initialise in addition to actually initialise the object
   */
  HonkWaitRecovery();

  /**
   * @brief Initialisation function for the HonkWaitRecovery recovery behaviour
   * @param tf A pointer to a transform listener
   * @param global_costmap A pointer to the global_costmap used by the navigation stack
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name,
                  tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief Run the HonkWaitRecovery recovery behaviour
   *
   * Triggers a sound and waits for a specified time
   */
  void runBehavior();

private:
  bool initialised_;
  double waiting_duration_;
  ros::Publisher pub_honk_;
};

} // namespace

#endif /* HONK_WAIT_RECOVERY_H_ */
