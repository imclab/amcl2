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

#ifndef AMCL_WRAPPER_H
#define AMCL_WRAPPER_H

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// AMCL
#include <amcl/map/map.h>
#include <amcl/pf/pf.h>
#include <amcl/sensors/amcl_odom.h>
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
#include "amcl_ros/AmclConfig.h"

#include "amcl_ros/amcl_map_wrapper.h"
#include "amcl_ros/amcl_laser_wrapper.h"

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

namespace amcl_ros
{

class AmclWrapper
{
  public:
    AmclWrapper();
    ~AmclWrapper();

    pf_t* getParticleFilter() { return particle_filter_; }

    // check if we need to update our sensor data
    bool checkOdomDelta( ros::Time stamp, pf_vector_t &pose );

    // call motion prediction in particle filter
    void updateOdom( ros::Time stamp, pf_vector_t pose );

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    map_t *getMap() { return map_wrapper_.getMap(); }

    // resample particles based on their current weights
    // (every resample_interval_ times it is called)
    // and publish particle cloud
    void resample( );

    // publish the most recent tf transform between odom and map
    void publishTf( ros::Time time );

    // Pose-generating function used to uniformly distribute particles over
    // the map
    pf_vector_t uniformPoseGenerator();

    void onMapChanged();

  private:

    void initializeParticleFilter();

    // ROS Callbacks
    bool globalLocalizationCb(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);

    bool nomotionUpdateCb(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);

    void initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void dynamicReconfigureCb(amcl_ros::AmclConfig &config, uint32_t level);

    // reset the tf message filters, in case frame names have changed
    void resetTfMessageFilters();

    void resetSensorModels();

    void freeMapDependentMemory();

    void applyInitialPose();

    // indicate for all sensors if they need an update
    void resetUpdateFlags( bool need_update );

    double getYaw(tf::Pose& t);

    // Parameters ////////////////////////////////////

    // tf frame names
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;

    ros::Duration gui_publish_interval_;
    ros::Duration save_pose_interval_;
    int resample_interval_;

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    double kld_err_, kld_z_;
    double update_min_d_, update_min_a_;

    odom_model_t odom_model_type_;

    int min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double recovery_alpha_slow_, recovery_alpha_fast_;
    double initial_pose_[3];
    double initial_cov_[3];

    // ROS interface /////////////////////////////////////

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    boost::shared_ptr< tf::TransformBroadcaster > tf_broadcaster_;
    boost::shared_ptr< tf::TransformListener > tf_listener_;

    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_;

    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;

    ros::Subscriber initial_pose_sub_;

    // dynamic reconfigure
    boost::shared_ptr< dynamic_reconfigure::Server<amcl_ros::AmclConfig> > dyn_reconf_srv_;
    amcl_ros::AmclConfig default_config_;

    // map / sensor models /////////////////////////////////////

    pf_t* particle_filter_;

    boost::shared_ptr< AMCLOdom > odom_;

    boost::shared_ptr< amcl_hyp_t > initial_pose_hyp_;

    // other internals /////////////////////////////////////

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    int resample_count_;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose_;

    ros::Time lase_save_pose_time_;

    //used to temporarily let amcl update samples even when no motion occurs
    bool force_update_;

    bool force_pose_publication_;

    pf_vector_t last_update_odom_pose_;
    ros::Time last_update_odom_time_;
    bool last_update_odom_pose_initialized_;

    bool first_reconfigure_call_;

    // global mutex. should be locked within any ros callback
    boost::recursive_mutex global_mutex_;

    AmclMapWrapper map_wrapper_;

    AmclLaserWrapper laser_wrapper_;
};

}

#endif
