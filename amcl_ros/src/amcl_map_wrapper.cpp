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

#include "amcl_ros/amcl_map_wrapper.h"

#include "nav_msgs/GetMap.h"

namespace amcl_ros
{


AmclMapWrapper::AmclMapWrapper( std::string ns ) :
    private_nh_(ns),
    map_(0)
{
}

void AmclMapWrapper::init()
{
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &AmclMapWrapper::mapCb, this);
    ROS_INFO("Subscribed to map topic.");
  } else {
    requestMap();
  }
}

AmclMapWrapper::~AmclMapWrapper()
{

}

pf_vector_t AmclMapWrapper::uniformPoseGenerator()
{
  unsigned int rand_index = drand48() * free_space_coords_.size();
  std::pair<float,float> free_point = free_space_coords_[rand_index];
  pf_vector_t p;
  p.v[0] = free_point.first;
  p.v[1] = free_point.second;
  p.v[2] = drand48() * 2 * M_PI - M_PI;
  return p;
}

void
AmclMapWrapper::requestMap()
{
  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void AmclMapWrapper::mapCb(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}


void AmclMapWrapper::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();

  map_ = convertMap(msg);

  // Index of free space
  free_space_coords_.reserve( map_->size_x*map_->size_y );
  for(int i = 0; i < map_->size_x; i++)
  {
    for(int j = 0; j < map_->size_y; j++)
    {
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
      {
        float x = MAP_WXGX(map_,i);
        float y = MAP_WYGY(map_,j);
        free_space_coords_.push_back(std::make_pair(x,y));
      }
    }
  }

  mapChanged();
}


map_t* AmclMapWrapper::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}


void
AmclMapWrapper::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
}


}

