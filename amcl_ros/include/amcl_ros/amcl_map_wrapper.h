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

#ifndef AMCL_MAP_WRAPPER_H
#define AMCL_MAP_WRAPPER_H


#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <string>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/signals2.hpp>

// AMCL
#include <amcl/map/map.h>
#include <amcl/pf/pf.h>

#include "ros/ros.h"

#include "nav_msgs/GetMap.h"

// Dynamic_reconfigure
#include "amcl_ros/AmclConfig.h"

namespace amcl_ros
{

class AmclMapWrapper
{
  public:
    AmclMapWrapper( std::string ns );
    ~AmclMapWrapper();

    void init();

    void setUseMapTopic( bool b ) { use_map_topic_ = b; }
    void setFirstMapOnly( bool b ) { first_map_only_ = b; }

    // signals:
    boost::signals2::signal<void() > mapChanged;

    map_t *getMap() { return map_; }

    // Pose-generating function used to uniformly distribute particles over
    // the map
    pf_vector_t uniformPoseGenerator();

  private:

    void freeMapDependentMemory();

    void mapCb(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);

    // call GetMap service
    void requestMap();

    /**
     * Convert an OccupancyGrid map message into the internal
     * representation.  This allocates a map_t and returns it.
     */
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );

    // Parameters ////////////////////////////////////

    bool use_map_topic_;
    bool first_map_only_;

    bool first_map_received_;

    // ROS interface /////////////////////////////////////

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber map_sub_;

    map_t* map_;

    ////////////////////////////

    std::vector<std::pair<float,float> > free_space_coords_;
};

}


#endif
