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


#ifndef DENSE_LASER_ASSEMBLER_DENSE_LASER_MSG_FILTER_H_
#define DENSE_LASER_ASSEMBLER_DENSE_LASER_MSG_FILTER_H_

#include "ros/ros.h"
#include "dense_laser_assembler/joint_pv_msg_filter.h"
#include "dense_laser_assembler/laser_scan_tagger.h"
#include "message_filters/cache.h"
#include "message_filters/connection.h"
#include "message_filters/simple_filter.h"


// Messages
#include "sensor_msgs/LaserScan.h"
#include "mechanism_msgs/MechanismState.h"
#include "calibration_msgs/DenseLaserSnapshot.h"

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

namespace dense_laser_assembler
{

/**
 * \brief Listens to LaserScan and MechanismState messages, and builds DenseLaserSnapshots
 */
class DenseLaserMsgFilter : public message_filters::SimpleFilter<TaggedLaserScan<JointPVArray> >
{
public:

  typedef boost::shared_ptr<const TaggedLaserScan<JointPVArray> > MConstPtr ;

  //! \brief Not yet implemented
  template<class A, class B>
  DenseLaserMsgFilter(std::string name, A& a, B&b, unsigned int laser_queue_size,
                      unsigned int laser_cache_size, unsigned int mech_state_cache,
                      std::vector<std::string> joint_names) ;


  /**
   * \brief Construct assembler, and subscribe to the both LaserScan and MechanismState data providers
   * \param name Namespace for the node. Filter will search for params in ~/[name]/
   * \param laser_scan_provider MsgFilter that will provide LaserScan messages
   * \param mech_state_provider MsgFilter that will provide MechanismState message
   */
  template<class A, class B>
  DenseLaserMsgFilter(std::string name, A& laser_scan_provider, B& mech_state_provider)
  {
    subscribeLaserScan(laser_scan_provider) ;
    subscribeMechState(mech_state_provider) ;

    // Get all the different queue size parameters
    CONSTRUCT_INT(laser_queue_size, 40) ;
    CONSTRUCT_INT(laser_cache_size, 1000) ;
    CONSTRUCT_INT(mech_state_cache_size, 100) ;

    // Get the list of joints that we care about
    joint_names_.clear() ;
    bool found_joint = true ;
    int joint_count = 0 ;

    char param_buf[1024] ;
    while(found_joint)
    {
      sprintf(param_buf, "~joint_name_%02u", joint_count) ;
      std::string param_name = param_buf ;
      std::string cur_joint_name ;
      found_joint = n_.getParam(param_name, cur_joint_name) ;
      if (found_joint)
      {
        ROS_INFO("[%s] -> %s", param_name.c_str(), cur_joint_name.c_str()) ;
        joint_names_.push_back(cur_joint_name) ;
      }
      else
        ROS_DEBUG("Did not find param [%s]", param_name.c_str()) ;
      joint_count++ ;
    }

    // Configure the joint_pv_filter and associated cache
    joint_pv_filter_.setJointNames(joint_names_) ;
    joint_cache_.setCacheSize(mech_state_cache_size) ;
    joint_cache_.connectInput(joint_pv_filter_) ;

    // Set up the laser tagger and associated cache
    laser_tagger_.setMaxQueueSize(laser_queue_size) ;
    laser_tagger_.subscribeTagCache(joint_cache_) ;
    tagged_laser_cache_.setCacheSize(laser_cache_size) ;
    tagged_laser_cache_.connectInput(laser_tagger_) ;
  }

  //! \brief Not yet implemented
  void processLaserScan(const sensor_msgs::LaserScanConstPtr& msg) ;

  //! \brief Not yet implemented
  void processMechState(const mechanism_msgs::MechanismState& msg) ;

  //! \brief Get what the oldest time processed scan is
  ros::Time getOldestScanTime() ;
  //! \brief Get what the new time processed scan is
  ros::Time getNewstScanTime() ;

  /**
   * \brief Builds a dense laser snapshot
   * Grabs all the scans between the start and end time, and composes them into one larger snapshot. This call is non-blocking.
   * Thus, if there are scans that haven't been cached yet, but occur before the end time, they won't be added to the snapshot.
   * \param start The earliest scan time to be included in the snapshot
   * \param end The latest scan time to be included in the snapshot
   * \param output: A populated snapshot message
   * \return True if successful. False if unsuccessful
   */
  bool buildSnapshotFromInterval(const ros::Time& start, const ros::Time& end, calibration_msgs::DenseLaserSnapshot& snapshot) ;
private:
  ros::NodeHandle n_ ;

  /**
   * \brief Subscribe to a message filter that outputs LaserScans.
   * This is only useful if we're not subscribing to laser scans in the constructor.
   * Thus, this will stay private until we have more flexible constructors
   */
  template<class A>
  void subscribeLaserScan(A& a)
  {
    laser_tagger_.subscribeLaserScan(a) ;
  }

  /**
   * \brief Subscribe to a message filter that outputs MechanismState.
   * This is only useful if we're not subscribing to MechanismState in the constructor.
   * Thus, this will stay private until we have more flexible constructors
   */
  template<class B>
  void subscribeMechState(B& b)
  {
    joint_pv_filter_.connectInput(b) ;
  }

  //! Extracts positions data for a subset of joints in MechanismState
  JointPVMsgFilter joint_pv_filter_ ;

  //! Stores a time history of position data for a subset of joints in MechanismState
  message_filters::Cache<JointPVArray> joint_cache_ ;

  //! Combines a laser scan with the set of pertinent joint positions
  LaserScanTagger< JointPVArray > laser_tagger_ ;

  //! Stores a time history of laser scans that are annotated with joint positions.
  message_filters::Cache< TaggedLaserScan<JointPVArray> > tagged_laser_cache_ ;

  std::vector<std::string> joint_names_ ;
} ;

}

#undef CONSTRUCT_INT

#endif
