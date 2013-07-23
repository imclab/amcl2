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

#include "amcl_ros/amcl_wrapper.h"

namespace amcl_ros
{

static double normalize(double z)
{
  return atan2(sin(z),cos(z));
}

static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

pf_vector_t uniformPoseGenerator( AmclWrapper *w )
{
  return w->uniformPoseGenerator();
}

AmclWrapper::AmclWrapper() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        particle_filter_(NULL),
        resample_count_(0),
        private_nh_("~"),
        first_reconfigure_call_(true),
        force_pose_publication_(false),
        force_update_(false),
        laser_wrapper_(this, "~/laser"),
        map_wrapper_("~")
{
  boost::recursive_mutex::scoped_lock l(global_mutex_);

  // Grab params off the param server

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_interval_ = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_interval_ = ros::Duration(1.0/tmp);

  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", kld_err_, 0.01);
  private_nh_.param("kld_z", kld_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);

  std::string tmp_model_type;
  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", update_min_d_, 0.2);
  private_nh_.param("update_min_a", update_min_a_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("resample_interval", resample_interval_, 2);

  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  transform_tolerance_.fromSec(tmp_tol);
  private_nh_.param("recovery_alpha_slow", recovery_alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", recovery_alpha_fast_, 0.1);

  initial_pose_[0] = 0.0;
  initial_pose_[1] = 0.0;
  initial_pose_[2] = 0.0;
  initial_cov_[0] = 0.5 * 0.5;
  initial_cov_[1] = 0.5 * 0.5;
  initial_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, initial_pose_[0]);
  if(!std::isnan(tmp_pos))
    initial_pose_[0] = tmp_pos;
  else 
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, initial_pose_[1]);
  if(!std::isnan(tmp_pos))
    initial_pose_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, initial_pose_[2]);
  if(!std::isnan(tmp_pos))
    initial_pose_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");
  private_nh_.param("initial_cov_xx", tmp_pos, initial_cov_[0]);
  if(!std::isnan(tmp_pos))
    initial_cov_[0] =tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, initial_cov_[1]);
  if(!std::isnan(tmp_pos))
    initial_cov_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, initial_cov_[2]);
  if(!std::isnan(tmp_pos))
    initial_cov_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");

  // setup ROS publishers, subscribers, services

  tf_broadcaster_.reset( new tf::TransformBroadcaster() );
  tf_listener_.reset( new tf::TransformListener() );

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  global_loc_srv_ = nh_.advertiseService("global_localization", 
					 &AmclWrapper::globalLocalizationCb,
                                         this);
  nomotion_update_srv_= nh_.advertiseService("request_nomotion_update", &AmclWrapper::nomotionUpdateCb, this);

  // dynamic reconfigure
  dyn_reconf_srv_.reset( new dynamic_reconfigure::Server<amcl_ros::AmclConfig>(ros::NodeHandle("~")) );
  dynamic_reconfigure::Server<amcl_ros::AmclConfig>::CallbackType cb = boost::bind(&AmclWrapper::dynamicReconfigureCb, this, _1, _2);
  dyn_reconf_srv_->setCallback(cb);
  dyn_reconf_srv_->getConfigDefault(default_config_);

  resetTfMessageFilters();

  map_wrapper_.mapChanged.connect( boost::bind( &AmclWrapper::resetSensorModels, this ) );
  map_wrapper_.mapChanged.connect( boost::bind( &AmclWrapper::initializeParticleFilter, this ) );
  map_wrapper_.mapChanged.connect( boost::bind( &AmclWrapper::applyInitialPose, this ) );
  map_wrapper_.init();
}

AmclWrapper::~AmclWrapper()
{
  freeMapDependentMemory();
}


void AmclWrapper::resetTfMessageFilters()
{
  // this does a transformation in global_frame_id_ and should be recalled here
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclWrapper::initialPoseCb, this);
}

void AmclWrapper::resetSensorModels()
{
  // Instantiate the sensor objects
  odom_.reset( new AMCLOdom() );
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );

  laser_wrapper_.resetSensorModel();
}

void AmclWrapper::dynamicReconfigureCb(AmclConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(global_mutex_);

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

  update_min_d_ = config.update_min_d;
  update_min_a_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  gui_publish_interval_ = ros::Duration(1.0/config.gui_publish_rate);
  save_pose_interval_ = ros::Duration(1.0/config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  if(config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be less than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  recovery_alpha_slow_ = config.recovery_alpha_slow;
  recovery_alpha_fast_ = config.recovery_alpha_fast;

  particle_filter_ = pf_alloc(min_particles_, max_particles_,
                 recovery_alpha_slow_, recovery_alpha_fast_,
                 reinterpret_cast<pf_init_model_fn_t>(&amcl_ros::uniformPoseGenerator),
                 (void *)this);

  kld_err_ = config.kld_err;
  kld_z_ = config.kld_z;
  particle_filter_->pop_err = kld_err_;
  particle_filter_->pop_z = kld_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose_.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose_.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose_.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose_.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose_.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose_.pose.covariance[6*5+5];
  pf_init(particle_filter_, pf_init_pose_mean, pf_init_pose_cov);
  last_update_odom_pose_initialized_ = false;

  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;

  resetTfMessageFilters();
  resetSensorModels();
}

void
AmclWrapper::freeMapDependentMemory()
{
  if( particle_filter_ != NULL ) {
    pf_free( particle_filter_ );
    particle_filter_ = NULL;
  }
  odom_.reset();
}

bool
AmclWrapper::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_listener_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}


pf_vector_t
AmclWrapper::uniformPoseGenerator()
{
  return map_wrapper_.uniformPoseGenerator();
}

bool
AmclWrapper::globalLocalizationCb(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  boost::recursive_mutex::scoped_lock ml(global_mutex_);

  if( map_wrapper_.getMap() == NULL ) {
    return true;
  }

  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(particle_filter_,
                reinterpret_cast<pf_init_model_fn_t>(&amcl_ros::uniformPoseGenerator),
                (void *)this);
  ROS_INFO("Global initialisation done!");
  last_update_odom_pose_initialized_ = false;
  return true;
}

bool 
AmclWrapper::nomotionUpdateCb(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  boost::recursive_mutex::scoped_lock ml(global_mutex_);
  force_update_ = true;
  //ROS_INFO("Requesting no-motion update");
  return true;
}

void
AmclWrapper::resetUpdateFlags( bool need_update )
{
  laser_wrapper_.resetUpdateFlags( need_update );
}

void AmclWrapper::initializeParticleFilter()
{
  // Create the particle filter
  particle_filter_ = pf_alloc(min_particles_, max_particles_,
                 recovery_alpha_slow_, recovery_alpha_fast_,
                 (pf_init_model_fn_t)amcl_ros::uniformPoseGenerator,
                 (void *)this);
  particle_filter_->pop_err = kld_err_;
  particle_filter_->pop_z = kld_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = initial_pose_[0];
  pf_init_pose_mean.v[1] = initial_pose_[1];
  pf_init_pose_mean.v[2] = initial_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = initial_cov_[0];
  pf_init_pose_cov.m[1][1] = initial_cov_[1];
  pf_init_pose_cov.m[2][2] = initial_cov_[2];
  pf_init(particle_filter_, pf_init_pose_mean, pf_init_pose_cov);
  last_update_odom_pose_initialized_ = false;
}

bool
AmclWrapper::checkOdomDelta( ros::Time stamp, pf_vector_t &pose )
{
  if ( last_update_odom_time_ > stamp )
  {
    ROS_ERROR("Detected negative time step in the sensor data.");
    return false;
  }

  if(!last_update_odom_pose_initialized_)
  {
    //initialize odometry pose tracking
    last_update_odom_pose_ = pose;
    last_update_odom_pose_initialized_ = true;
    force_pose_publication_ = true;
    resetUpdateFlags( true );
    resample_count_ = 0;
  }
  else
  {
    // Compute change in pose
    pf_vector_t delta = pf_vector_zero();
    delta.v[0] = pose.v[0] - last_update_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - last_update_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], last_update_odom_pose_.v[2]);

    // See if we should update the filter
    bool do_update = fabs(delta.v[0]) > update_min_d_ ||
                  fabs(delta.v[1]) > update_min_d_ ||
                  fabs(delta.v[2]) > update_min_a_;
    do_update = do_update || force_update_;
    force_update_ = false;
    force_pose_publication_ = true;

    // Set the laser update flags
    if(do_update)
    {
      resetUpdateFlags( true );
    }
  }

  return true;
}

void
AmclWrapper::updateOdom( ros::Time stamp, pf_vector_t pose )
{
  // Compute change in pose
  pf_vector_t delta = pf_vector_zero();
  delta.v[0] = pose.v[0] - last_update_odom_pose_.v[0];
  delta.v[1] = pose.v[1] - last_update_odom_pose_.v[1];
  delta.v[2] = angle_diff(pose.v[2], last_update_odom_pose_.v[2]);

  // If the robot has moved, update the filter
  AMCLOdomData odom_data;
  odom_data.pose = pose;
  odom_data.delta = delta;
  odom_->UpdateAction(particle_filter_, (AMCLSensorData*)&odom_data);

  last_update_odom_time_ = stamp;
  last_update_odom_pose_ = pose;
}

void
AmclWrapper::publishTf( ros::Time time )
{
  if(latest_tf_valid_)
  {
    ros::Time transform_expiration = time + transform_tolerance_;
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_id_, odom_frame_id_);
    this->tf_broadcaster_->sendTransform(tmp_tf_stamped);

    sent_first_transform_ = true;

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_interval_.toSec() > 0.0) &&
       (now - lase_save_pose_time_) >= save_pose_interval_)
    {
      // We need to apply the last transform to the latest odom pose to get
      // the latest map pose to store.  We'll take the covariance from
      // last_published_pose.
      tf::Pose odom_pose;
      odom_pose.getOrigin().setX(last_update_odom_pose_.v[0]);
      odom_pose.getOrigin().setY(last_update_odom_pose_.v[1]);
      odom_pose.getBasis().setRPY(0,0,last_update_odom_pose_.v[2]);
      tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
      double yaw,pitch,roll;
      map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

      private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
      private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
      private_nh_.setParam("initial_pose_a", yaw);
      private_nh_.setParam("initial_cov_xx",
                                      last_published_pose_.pose.covariance[6*0+0]);
      private_nh_.setParam("initial_cov_yy",
                                      last_published_pose_.pose.covariance[6*1+1]);
      private_nh_.setParam("initial_cov_aa",
                                      last_published_pose_.pose.covariance[6*5+5]);
      lase_save_pose_time_ = now;
    }
  }
}

void
AmclWrapper::resample( )
{
  bool resampled = false;

  // Resample the particles
  if(!(++resample_count_ % resample_interval_))
  {
    pf_update_resample(particle_filter_);
    resampled = true;
  }

  pf_sample_set_t* set = particle_filter_->sets + particle_filter_->current_set;
  ROS_DEBUG("Num samples: %d\n", set->sample_count);

  // Publish the resulting cloud
  // TODO: set maximum rate for publishing
  if (!force_update_) {
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(set->sample_count);
    for(int i=0;i<set->sample_count;i++)
    {
      tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                               tf::Vector3(set->samples[i].pose.v[0],
                                         set->samples[i].pose.v[1], 0)),
                      cloud_msg.poses[i]);
    }
    particlecloud_pub_.publish(cloud_msg);
  }

  if( resampled || force_pose_publication_ )
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(particle_filter_->sets[particle_filter_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < particle_filter_->sets[particle_filter_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(particle_filter_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = last_update_odom_time_;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = particle_filter_->sets + particle_filter_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

      pose_pub_.publish(p);
      last_published_pose_ = p;

      ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              last_update_odom_time_,
                                              base_frame_id_);
        this->tf_listener_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
}


double
AmclWrapper::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void
AmclWrapper::initialPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock prl(global_mutex_);

  if(msg->header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_listener_->resolve(msg->header.frame_id) != tf_listener_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg->header.frame_id.c_str(),
             global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    tf_listener_->lookupTransform(base_frame_id_, ros::Time::now(),
                         base_frame_id_, msg->header.stamp,
                         global_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg->pose.pose, pose_old);
  pose_new = tx_odom.inverse() * pose_old;

  // Transform into the global frame

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg->pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg->pose.covariance[6*5+5];

  initial_pose_hyp_.reset( new amcl_hyp_t() );
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void
AmclWrapper::applyInitialPose()
{
  if( initial_pose_hyp_ != NULL && particle_filter_ != NULL ) {
    pf_init(particle_filter_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    last_update_odom_pose_initialized_ = false;

    initial_pose_hyp_.reset();
  }
}


}
