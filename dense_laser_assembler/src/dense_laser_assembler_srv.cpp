/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//! \author Vijay Pradeep / vpradeep@willowgarage.com

#include "ros/ros.h"
#include "dense_laser_assembler/build_dense_laser_snapshot.h"
#include "dense_laser_assembler/joint_extractor.h"
#include "dense_laser_assembler/laser_scan_tagger.h"
#include "message_filters/msg_cache.h"

//#include "dense_laser_assembler/test_func.h"

// Messages/Services
#include "dense_laser_assembler/BuildLaserSnapshot.h"

#define CONSTRUCT_INT(param, default_val) \
  int param ;\
  if (!n_.getParam("~" #param, param) ) \
  { \
    ROS_WARN("[~" #param "] not set. Setting to default value of [%u]", default_val) ; \
    param = default_val ; \
  } \
  else \
  { \
    ROS_INFO("[~" #param "] set to value of [%u]", param) ; \
  }

using namespace dense_laser_assembler ;
using namespace std ;

class DenseLaserAssemblerSrv
{
public:
  DenseLaserAssemblerSrv()
  {
    CONSTRUCT_INT(laser_queue_size, 40) ;
    CONSTRUCT_INT(laser_cache_size, 1000) ;
    CONSTRUCT_INT(mech_state_cache_size, 100) ;

    bool found_joint = true ;
    int joint_count = 0 ;

    char param_buf[1024] ;
    while(found_joint)
    {
      sprintf(param_buf, "~joint_name_%02u", joint_count) ;
      string param_name = param_buf ;
      string cur_joint_name ;
      found_joint = n_.getParam(param_name, cur_joint_name) ;
      if (found_joint)
      {
        ROS_INFO("[%s] -> %s\n", param_name.c_str(), cur_joint_name.c_str()) ;
        joint_names_.push_back(cur_joint_name) ;
      }
    }


    // Configure the joint extractor
    joint_extractor_.setJointNames(joint_names_) ;
    joint_cache_.setCacheSize(mech_state_cache_size) ;
    joint_cache_.subscribe(joint_extractor_) ;

    // Set up the laser tagger (and link it to the joint_cache)
    laser_tagger_.setTagCache(joint_cache_) ;
    laser_tagger_.setQueueSize(laser_queue_size) ;

    // Trigger laser_tagger off updates to the joint_cache
    joint_cache_.addOutputCallback(boost::bind(&LaserJointTagger::update, &laser_tagger_)) ;

    // Set up the cache for tagged laser scans (and link it to the laser_tagger)
    tagged_laser_cache_.setCacheSize(laser_cache_size) ;
    tagged_laser_cache_.subscribe(laser_tagger_) ;

    // Link the joint extractor to a topic
    mech_sub_  = n_.subscribe("mechanism_state", 1, &JointExtractor::processMechState, &joint_extractor_) ;

    // Link the laser_tagger to a topic
    laser_sub_ = n_.subscribe("tilt_scan", 1, &LaserJointTagger::processLaserScan, &laser_tagger_) ;

    // Advertise services
    snapshot_adv_ = n_.advertiseService("~build_laser_snapshot", &DenseLaserAssemblerSrv::buildLaserSnapshotSrv, this) ;
  }

  bool buildLaserSnapshotSrv(BuildLaserSnapshot::Request& req, BuildLaserSnapshot::Response& resp)
  {
    vector<boost::shared_ptr<const JointTaggedLaserScan> > scans = tagged_laser_cache_.getInterval(req.start, req.end) ;
    bool success = true ;
    success = buildDenseLaserSnapshot(scans, joint_names_, resp.snapshot) ;

    /*printf("Displaying times:\n") ;
    for (unsigned int i=0; i<resp.snapshot.scan_start.size(); i++)
    {
      printf("   %u: %f\n", i, resp.snapshot.scan_start[i].toSec()) ;
    }*/

    return success ;
  }

private:
  ros::NodeHandle n_ ;
  ros::Subscriber mech_sub_ ;
  ros::Subscriber laser_sub_ ;
  ros::ServiceServer snapshot_adv_ ;

  //! Extracts positions data for a subset of joints in MechanismState
  JointExtractor joint_extractor_ ;
  //! Stores a time history of position data for a subset of joints in MechanismState
  message_filters::MsgCache<JointExtractor::JointArray> joint_cache_ ;

  //! Combines a laser scan with the set of pertinent joint positions
  LaserJointTagger laser_tagger_ ;

  //! Stores a time history of laser scans that are annotated with joint positions.
  JointTaggedLaserScanCache tagged_laser_cache_ ;

  vector<string> joint_names_ ;
} ;




int main(int argc, char** argv)
{
  ros::init(argc, argv, "dense_laser_assembler_srv") ;

  DenseLaserAssemblerSrv assembler ;

  ros::spin() ;
}
