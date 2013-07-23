/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey, David Gossow */

#include "amcl_ros/amcl_laser_wrapper.h"
#include "amcl_ros/amcl_wrapper.h"

static const std::string scan_topic_ = "scan";

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// AMCL
#include <amcl/map/map.h>
#include <amcl/pf/pf.h>
#include <amcl/sensors/amcl_laser.h>

// roscpp
#include "ros/assert.h"
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"

namespace amcl_ros
{

AmclLaserWrapper::AmclLaserWrapper( AmclWrapper* parent, std::string ns ) :
            private_nh_(ns),
            first_reconfigure_call_(true),
            force_update_(false),
            laser_needs_update_(false)
{
  parent_ = parent;

  ros::NodeHandle parent_private_nh("~");

  parent_private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  parent_private_nh.param("base_frame_id", base_frame_id_, std::string("base_link"));

  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);

  private_nh_.param("laser_z_hit", laser_z_hit_, 0.95);
  private_nh_.param("laser_z_short", laser_z_short_, 0.1);
  private_nh_.param("laser_z_max", laser_z_max_, 0.05);
  private_nh_.param("laser_z_rand", laser_z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", laser_sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", laser_lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);

  std::string tmp_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  // setup ROS publishers, subscribers, services

  tf_listener_.reset( new tf::TransformListener() );

  laser_scan_sub_.reset( new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100) );

  // dynamic reconfigure
  dyn_reconf_srv_.reset( new dynamic_reconfigure::Server<AmclLaserConfig>(private_nh_) );
  dynamic_reconfigure::Server<AmclLaserConfig>::CallbackType cb = boost::bind(&AmclLaserWrapper::dynamicReconfigureCb, this, _1, _2);
  dyn_reconf_srv_->setCallback(cb);


  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                       boost::bind(&AmclLaserWrapper::checkLaserCb, this, _1));
  last_laser_received_time_ = ros::Time::now();

  resetTfMessageFilters();
}

AmclLaserWrapper::~AmclLaserWrapper()
{
  freeMapDependentMemory();
}

void
AmclLaserWrapper::freeMapDependentMemory()
{
  laser_.reset();
}

void
AmclLaserWrapper::checkLaserCb(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_time_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}



void AmclLaserWrapper::dynamicReconfigureCb(AmclLaserConfig &config, uint32_t level)
{
  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;

  max_beams_ = config.laser_max_beams;

  laser_z_hit_ = config.laser_z_hit;
  laser_z_short_ = config.laser_z_short;
  laser_z_max_ = config.laser_z_max;
  laser_z_rand_ = config.laser_z_rand;
  laser_sigma_hit_ = config.laser_sigma_hit;
  laser_lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if(config.laser_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(config.laser_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;

  resetTfMessageFilters();
  resetSensorModel();
}

void AmclLaserWrapper::resetTfMessageFilters()
{
  laser_scan_filter_.reset(
      new tf::MessageFilter<sensor_msgs::LaserScan>(*(laser_scan_sub_.get()),
                                                    *(tf_listener_.get()),
                                                    odom_frame_id_,
                                                    100) );
  laser_scan_filter_->registerCallback(boost::bind(&AmclLaserWrapper::laserCb,
                                                   this, _1));
}

void AmclLaserWrapper::resetSensorModel()
{
  if ( ! parent_->getMap() )
  {
    ROS_ERROR("Cannot reset sensor model: no map loaded.");
    return;
  }
  laser_.reset( new AMCLLaser(max_beams_, parent_->getMap()) );
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(laser_z_hit_, laser_z_short_, laser_z_max_, laser_z_rand_,
                         laser_sigma_hit_, laser_lambda_short_, 0.0);
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(laser_z_hit_, laser_z_rand_, laser_sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
}


void
AmclLaserWrapper::laserCb(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  last_laser_received_time_ = ros::Time::now();

  // First time we receive a laser message?
  if(!laser_.get())
  {
    ROS_DEBUG("Setting up laser (frame_id=%s)\n", laser_scan->header.frame_id.c_str());
    laser_ = boost::shared_ptr<AMCLLaser>( new AMCLLaser(*laser_) );
    laser_needs_update_ = true;
  }

  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)),
                               ros::Time(), laser_scan->header.frame_id);
  tf::Stamped<tf::Pose> laser_pose;
  try
  {
    this->tf_listener_->transformPose(base_frame_id_, ident, laser_pose);
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR("Couldn't transform from %s to %s, "
              "even though the message notifier is in use",
              laser_scan->header.frame_id.c_str(),
              base_frame_id_.c_str());
    return;
  }

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.getOrigin().x();
  laser_pose_v.v[1] = laser_pose.getOrigin().y();
  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  laser_->SetLaserPose(laser_pose_v);
  ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
            laser_pose_v.v[0],
            laser_pose_v.v[1],
            laser_pose_v.v[2]);

  // Where was the robot when this scan was taken?
  tf::Stamped<tf::Pose> odom_pose;
  pf_vector_t pose;
  if(!parent_->getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  // check if we need to update all sensors
  if ( !parent_->checkOdomDelta( laser_scan->header.stamp, pose ) )
  {
    // something bad happened
    return;
  }

  // If the robot has moved since the last time we used this laser,
  // update particle filter
  if(laser_needs_update_)
  {
    AMCLLaserData ldata;
    ldata.sensor = laser_.get();
    ldata.range_count = laser_scan->ranges.size();

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                      laser_scan->header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                      laser_scan->header.frame_id);
    try
    {
      tf_listener_->transformQuaternion(base_frame_id_, min_q, min_q);
      tf_listener_->transformQuaternion(base_frame_id_, inc_q, inc_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    ROS_DEBUG("Laser angles in base frame: min: %.3f inc: %.3f", angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if(laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
    else
      range_min = laser_scan->range_min;

    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for(int i=0;i<ldata.range_count;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min +
              (i * angle_increment);
    }

    // update odometry in particle filter
    parent_->updateOdom( laser_scan->header.stamp, pose );

    // update measurement
    laser_->UpdateSensor(parent_->getParticleFilter(), (AMCLSensorData*)&ldata);

    //resample
    parent_->resample();

    laser_needs_update_ = false;
  }

  parent_->publishTf( laser_scan->header.stamp );
}

}
