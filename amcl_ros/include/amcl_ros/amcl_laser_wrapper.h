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

#ifndef AMCL_LASER_WRAPPER_H
#define AMCL_LASER_WRAPPER_H

#include <string>

// AMCL
#include <amcl/map/map.h>
#include <amcl/pf/pf.h>
#include <amcl/sensors/amcl_laser.h>

// roscpp
#include "ros/assert.h"
#include "ros/ros.h"

// Messages
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
#include "amcl_ros/AmclLaserConfig.h"

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

namespace amcl_ros
{

class AmclWrapper;

class AmclLaserWrapper
{
  public:

    AmclLaserWrapper( AmclWrapper* parent, std::string ns );
    ~AmclLaserWrapper();

    // indicate for all sensors if they need an update
    void resetUpdateFlags( bool need_update ) { laser_needs_update_ = need_update; }

    void resetSensorModel();

  private:

    void laserCb(const sensor_msgs::LaserScanConstPtr& laser_scan);

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);

    void requestMap();

    void dynamicReconfigureCb(AmclLaserConfig &config, uint32_t level);

    void checkLaserCb(const ros::TimerEvent& event);

    // internal methods

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);

    // reset the tf message filters, in case frame names have changed
    void resetTfMessageFilters();

    void freeMapDependentMemory();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    void setBaseFrameId( std::string id ) { base_frame_id_ = id; }

    // Parameters ////////////////////////////////////

    int max_beams_;

    double laser_min_range_;
    double laser_max_range_;

    // tf frame names
    std::string base_frame_id_;
    std::string odom_frame_id_;

    ros::Duration laser_check_interval_;

    laser_model_t laser_model_type_;

    double laser_z_hit_,
      laser_z_short_,
      laser_z_max_,
      laser_z_rand_,
      laser_sigma_hit_,
      laser_lambda_short_;

    double laser_likelihood_max_dist_;

    // ROS interface /////////////////////////////////////

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // laser
    boost::shared_ptr< message_filters::Subscriber<sensor_msgs::LaserScan> > laser_scan_sub_;
    boost::shared_ptr< tf::MessageFilter<sensor_msgs::LaserScan> > laser_scan_filter_;

    // dynamic reconfigure
    boost::shared_ptr< dynamic_reconfigure::Server<amcl_ros::AmclLaserConfig> > dyn_reconf_srv_;
    amcl_ros::AmclLaserConfig default_config_;

    boost::shared_ptr< tf::TransformListener > tf_listener_;

    ros::Timer check_laser_timer_;

    // map / sensor models /////////////////////////////////////

    // grid map for laser-based localization

    boost::shared_ptr< AMCLLaser > laser_;
    bool laser_needs_update_;

    ros::Time last_laser_received_time_;

    //used to temporarily let amcl update samples even when no motion occurs
    bool force_update_;

    bool first_reconfigure_call_;

    AmclWrapper* parent_;

    std::vector<std::pair<int,int> > free_space_indices_;
};

}

#endif
