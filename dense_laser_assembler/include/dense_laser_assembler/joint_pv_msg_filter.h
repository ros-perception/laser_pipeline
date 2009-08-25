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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef DENSE_LASER_ASSEMBLER_JOINT_PV_MSG_FILTER_H_
#define DENSE_LASER_ASSEMBLER_JOINT_PV_MSG_FILTER_H_

#include <vector>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals.hpp>
#include <boost/bind.hpp>
#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>

// Messages
#include "dense_laser_assembler/JointPVArray.h"
#include "mechanism_msgs/MechanismState.h"

namespace dense_laser_assembler
{

/**
 * Streams an array of joint positions and velocities from MechanismState,
 * given a mapping from joint_names to array indices
 */
class JointPVMsgFilter : public message_filters::SimpleFilter<JointPVArray>
{
public:
  /**
   * \brief Subscribe to another MessageFilter at construction time
   * \param a The parent message filter
   * \param joint_names Vector of joint names that we want to output. Must match
   *                     joint names in mechanism state
   */
  template<class A>
  JointPVMsgFilter(A& a, std::vector<std::string> joint_names = std::vector<std::string>())
  {
    setJointNames(joint_names);
    subscribe(a);
  }

  /**
   * \brief Construct without subcribing to another MsgFilter at construction time
   * \param joint_names Vector of joint names that we want to output. Must match
   *                     joint names in mechanism state
   */
  JointPVMsgFilter(std::vector<std::string> joint_names = std::vector<std::string>())
  {
    setJointNames(joint_names);
  }

  /**
   * \brief Subcribes this MsgFilter to another MsgFilter
   * \param a The MsgFilter that this MsgFilter should get data from
   */
  template<class A>
  void connectInput(A& a)
  {
    incoming_connection_ = a.registerCallback(boost::bind(&JointPVMsgFilter::processMechState, this, _1));
  }

  /**
   * \brief Define the mapping from MechanismState to JointPVArray
   */
  void setJointNames(std::vector<std::string> joint_names)
  {
    boost::mutex::scoped_lock lock(joint_mapping_mutex_);
    joint_names_ = joint_names;
  }

  /**
   * \brief Extracts joint positions from MechState, using the joint_names mapping.
   * Also calls all the callback functions as set by connect()
   * \param msg The MechanismState message from which we want to extract positions
   */
  void processMechState(const mechanism_msgs::MechanismStateConstPtr& msg)
  {
    // Allocate mem to store out output data
    boost::shared_ptr<JointPVArray> joint_array_ptr(new JointPVArray) ;

    // Copy MechState data into a JointPVArray
    {
      boost::mutex::scoped_lock lock(joint_mapping_mutex_);

      joint_array_ptr->pos.resize(joint_names_.size()) ;
      joint_array_ptr->vel.resize(joint_names_.size()) ;

      joint_array_ptr->header = msg->header ;

      //! \todo This can seriously be optimized using a map, or some sort of cached lookup
      for (unsigned int i=0; i<joint_names_.size(); i++)
      {
        for (unsigned int j=0; j<msg->joint_states.size(); j++)
        {
          if (joint_names_[i] == msg->joint_states[j].name )
          {
            joint_array_ptr->pos[i] = msg->joint_states[j].position ;
            joint_array_ptr->vel[i] = msg->joint_states[j].velocity ;
            break ;
          }
        }
      }
    }

    // Call all connected callbacks
    signalMessage(joint_array_ptr) ;
  }

protected:
  boost::mutex joint_mapping_mutex_ ;
  std::vector<std::string> joint_names_ ;

  // Filter Connection Stuff
  message_filters::Connection incoming_connection_;
} ;

}

#endif // DENSE_LASER_ASSEMBLER_JOINT_PV_MSG_FILTER_H_
