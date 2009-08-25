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

#include "ros/ros.h"


#include "message_filters/subscriber.h"
#include "dense_laser_assembler/dense_laser_msg_filter.h"

// Messages
#include "calibration_msgs/DenseLaserSnapshot.h"
#include "pr2_msgs/LaserScannerSignal.h"


using namespace dense_laser_assembler ;

/**
 * \brief Builds a DenseLaserSnapshot message from laser scans collected in a specified time interval
 */
class DenseLaserSnapshotter
{

public:

  DenseLaserSnapshotter() :
    scan_sub_(n_, "tilt_scan", 20),
    mech_state_sub_(n_, "mechanism_state", 20),
    dense_laser_filter_("dense_laser_filter", scan_sub_, mech_state_sub_)
  {
    prev_signal_.header.stamp.fromNSec(0) ;
    signal_sub_ = n_.subscribe("laser_tilt_controller/laser_scanner_signal", 2, &DenseLaserSnapshotter::scannerSignalCallback, this) ;
    snapshot_pub_ = n_.advertise<calibration_msgs::DenseLaserSnapshot> ("dense_laser_snapshot", 1) ;
    first_time_ = true ;
  }

  ~DenseLaserSnapshotter()
  {

  }

  void scannerSignalCallback(const pr2_msgs::LaserScannerSignalConstPtr& cur_signal)
  {
    ROS_DEBUG("Got Scanner Signal") ;
    if (cur_signal->signal == 128 || cur_signal->signal == 129)       // These codes imply that this is the first signal during a given profile type
      first_time_ = true ;


    if (first_time_)
    {
      prev_signal_ = *cur_signal ;
      first_time_ = false ;
    }
    else
    {
      if (cur_signal->signal == 1)
      {
        ROS_DEBUG("About to make request") ;

        ros::Time start = prev_signal_.header.stamp ;
        ros::Time end = cur_signal->header.stamp ;

        calibration_msgs::DenseLaserSnapshot snapshot ;

        dense_laser_filter_.buildSnapshotFromInterval(start, end, snapshot) ;

        ROS_DEBUG("header.stamp: %f", snapshot.header.stamp.toSec()) ;
        ROS_DEBUG("header.frame_id: %s", snapshot.header.frame_id.c_str()) ;
        ROS_DEBUG("ranges.size()=%u", snapshot.ranges.size()) ;
        ROS_DEBUG("intensities.size()=%u", snapshot.intensities.size()) ;
        ROS_DEBUG("joint_positions.size()=%u", snapshot.joint_positions.size()) ;
        ROS_DEBUG("joint_velocities.size()=%u", snapshot.joint_velocities.size()) ;
        ROS_DEBUG("scan_start.size()=%u", snapshot.scan_start.size()) ;
        snapshot_pub_.publish(snapshot) ;

      }
      else
        ROS_DEBUG("Not making request") ;
      prev_signal_ = *cur_signal ;
    }
  }

private:
  ros::NodeHandle n_ ;
  ros::Publisher snapshot_pub_ ;
  ros::Subscriber signal_sub_ ;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_ ;
  message_filters::Subscriber<mechanism_msgs::MechanismState> mech_state_sub_ ;
  DenseLaserMsgFilter dense_laser_filter_ ;
  pr2_msgs::LaserScannerSignal prev_signal_;

  bool first_time_ ;
} ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dense_laser_snapshotter") ;
  DenseLaserSnapshotter snapshotter ;
  ros::spin() ;

  return 0;
}
