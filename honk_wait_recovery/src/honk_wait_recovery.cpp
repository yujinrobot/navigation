// License: Yujin

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <vector>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Empty.h>
#include "honk_wait_recovery/honk_wait_recovery.h"

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(honk_wait_recovery, HonkWaitRecovery, honk_wait_recovery::HonkWaitRecovery,
                        nav_core::RecoveryBehavior)

namespace honk_wait_recovery
{
HonkWaitRecovery::HonkWaitRecovery() : initialised_(false), waiting_duration_(2.0)
{
}

void HonkWaitRecovery::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap,
                                  costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialised_)
  {
    ros::NodeHandle nh_priv("~");

    nh_priv.param("waiting_duration", waiting_duration_, waiting_duration_);

    pub_honk_ = nh_priv.advertise<std_msgs::Empty>("honk", 1);

    initialised_ = true;
  }
  else
  {
    ROS_WARN_STREAM("You should not call initialise twice on this object. Doing nothing.");
  }
}

void HonkWaitRecovery::runBehavior()
{
  if (!initialised_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  // honk

  pub_honk_.publish(std_msgs::Empty());

  //
  ros::Time waiting_start = ros::Time::now();
  ros::Rate rate(0.1);
  ros::Duration waiting_duration(waiting_duration_);

  while (ros::ok())
  {
    if ((ros::Time::now() - waiting_start) > waiting_duration)
    {
      break;
    }
    rate.sleep();
  }
}

} // namespace
