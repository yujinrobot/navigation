// License: Yujin

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <vector>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Empty.h>
#include "clear_rotate_recovery/clear_rotate_recovery.h"

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(clear_rotate_recovery, ClearRotateRecovery, clear_rotate_recovery::ClearRotateRecovery,
                        nav_core::RecoveryBehavior)

namespace clear_rotate_recovery
{
ClearRotateRecovery::ClearRotateRecovery() : initialised_(false),
                                             global_costmap_(NULL),
                                             local_costmap_(NULL),
                                             tf_(NULL),
                                             sim_granularity_(0.01),
                                             min_rotational_vel_(0.1),
                                             max_rotational_vel_(1.0),
                                             acc_lim_th_(3.17),
                                             tolerance_(0.1),
                                             frequency_(10.0),
                                             reset_distance_(0.0),
                                             safe_rotation_(true),
                                             bumper_pressed_(false)
{
}

void ClearRotateRecovery::initialize(std::string name, tf::TransformListener* tf,
                                     costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialised_)
  {
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle nh_priv("~/" + name_);

    // clearing parameters
    nh_priv.param("reset_distance", reset_distance_, 3.0);
    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back(std::string("obstacles"));
    nh_priv.param("layer_names", clearable_layers, clearable_layers_default);
    for (unsigned i = 0; i < clearable_layers.size(); i++)
    {
      ROS_INFO("Recovery behaviour will clear layer %s", clearable_layers[i].c_str());
      clearable_layers_.insert(clearable_layers[i]);
    }

    // rotate parameters
    nh_priv.param("sim_granularity", sim_granularity_, sim_granularity_);
    nh_priv.param("frequency", frequency_, frequency_);
    nh_priv.param("acc_lim_th", acc_lim_th_, acc_lim_th_);
    nh_priv.param("max_rotational_vel", max_rotational_vel_, max_rotational_vel_);
    nh_priv.param("min_in_place_rotational_vel", min_rotational_vel_, min_rotational_vel_);
    nh_priv.param("yaw_goal_tolerance", tolerance_, tolerance_);
    nh_priv.param("safe_rotation", safe_rotation_, safe_rotation_);
    world_model_.reset(new base_local_planner::CostmapModel(*local_costmap_->getCostmap()));
    pub_vel_ = nh_priv.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    if (!safe_rotation_)
    {
      sub_bumper_ = nh_priv.subscribe("bumper", 1, &ClearRotateRecovery::bumperCB, this);
    }

    initialised_ = true;
  }
  else
  {
    ROS_WARN_STREAM("You should not call initialise twice on this object. Doing nothing.");
  }
}

void ClearRotateRecovery::runBehavior()
{
  if (!initialised_)
  {
    ROS_ERROR("This object must be initialised before runBehavior() is called");
    return;
  }

  if (global_costmap_ == NULL || local_costmap_ == NULL)
  {
    ROS_ERROR("The costmaps passed to the ClearRotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ROS_WARN("Clearing costmaps at a distance of %f m and larger.", reset_distance_);
  clear(global_costmap_);
  clear(local_costmap_);
  ROS_WARN("Starting rotation.");
  rotate();
}

void ClearRotateRecovery::clear(costmap_2d::Costmap2DROS* costmap)
{
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  tf::Stamped<tf::Pose> pose;

  if (!costmap->getRobotPose(pose))
  {
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.getOrigin().x();
  double y = pose.getOrigin().y();

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin();
      pluginp != plugins->end(); ++pluginp)
  {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if (slash != std::string::npos)
    {
      name = name.substr(slash + 1);
    }

    if (clearable_layers_.count(name) != 0)
    {
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
    }
  }
}

void ClearRotateRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y)
{
  boost::unique_lock<boost::shared_mutex> lock(*(costmap->getLock()));

  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  unsigned char* grid = costmap->getCharMap();
  for (int x = 0; x < (int)costmap->getSizeInCellsX(); x++)
  {
    bool xrange = x > start_x && x < end_x;

    for (int y = 0; y < (int)costmap->getSizeInCellsY(); y++)
    {
      if (xrange && y > start_y && y < end_y)
        continue;
      int index = costmap->getIndex(x, y);
      if (grid[index] != costmap_2d::NO_INFORMATION)
      {
        grid[index] = costmap_2d::NO_INFORMATION;
      }
    }
  }

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

void ClearRotateRecovery::rotate()
{
  ros::Rate r(frequency_);

  tf::Stamped < tf::Pose > global_pose;
  local_costmap_->getRobotPose(global_pose);
  double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  double current_angle = -1.0 * M_PI;
  bool got_180 = false;
  bumper_pressed_ = false;

  while (ros::ok())
  {
    local_costmap_->getRobotPose(global_pose);

    double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    current_angle = angles::normalize_angle(norm_angle + start_offset);

    //compute the distance left to rotate
    double dist_left = M_PI - current_angle;

    double x = global_pose.getOrigin().x(), y = global_pose.getOrigin().y();

    if (safe_rotation_)
    {
      //check if that velocity is legal by forward simulating
      double sim_angle = 0.0;
      while (sim_angle < dist_left)
      {
        double theta = tf::getYaw(global_pose.getRotation()) + sim_angle;

        //make sure that the point is legal, if it isn't... we'll abort
        double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
        if (footprint_cost < 0.0)
        {
          ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                    footprint_cost);
          return;
        }

        sim_angle += sim_granularity_;
      }
    }
    else // rotate unless we get
    {
      if (bumper_pressed_)
      {
        ROS_ERROR("Can't rotate in place because we bumped into something or someone!");
        return;
      }
    }

    //compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(2 * acc_lim_th_ * dist_left);

    //make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    pub_vel_.publish(cmd_vel);

    //makes sure that we won't decide we're done right after we start
    if (current_angle < 0.0)
      got_180 = true;

    //if we're done with our in-place rotation... then return
    if (got_180 && current_angle >= (0.0 - tolerance_))
      return;

    r.sleep();
  }
}

void ClearRotateRecovery::bumperCB(const std_msgs::EmptyConstPtr msg)
{
  bumper_pressed_ = true;
}

} // namespace
