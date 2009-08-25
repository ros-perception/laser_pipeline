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

#include "ros/node.h"

// Services
#include "point_cloud_assembler/BuildCloud.h"

// Messages
#include "sensor_msgs/PointCloud.h"
#include "pr2_msgs/LaserScannerSignal.h"

/***
 * This uses the point_cloud_assembler's build_cloud service call to grab all the scans/clouds between two tilt-laser shutters
 * params
 *  * "~target_frame_id" (string) - This is the frame that the scanned data transformed into.  The
 *                                  output clouds are also published in this frame.
 *  * "~num_skips" (int)          - If set to N>0, then the snapshotter will skip N signals before 
 *                                  requesting a new snapshot. This will make the snapshots be N times 
 *                                  larger. Default 0 - no skipping.
 */

namespace point_cloud_assembler
{

class PointCloudSnapshotter
{

public:

  pr2_msgs::LaserScannerSignal prev_signal_;
  pr2_msgs::LaserScannerSignal cur_signal_;

  bool first_time_ ;

  int num_skips_;
  int num_skips_left_;

  std::string fixed_frame_ ;
  ros::Node node_;

  PointCloudSnapshotter() : node_("point_cloud_snapshotter")
  {
    prev_signal_.header.stamp.fromNSec(0) ;

    ros::Node::instance()->advertise<sensor_msgs::PointCloud> ("full_cloud", 1) ;
    ros::Node::instance()->subscribe("laser_scanner_signal", cur_signal_, &PointCloudSnapshotter::scannerSignalCallback, this, 40) ;

    ros::Node::instance()->param("~num_skips", num_skips_, 0) ;
    num_skips_left_=num_skips_;

    first_time_ = true ;
  }

  ~PointCloudSnapshotter()
  {
    ros::Node::instance()->unsubscribe("laser_scanner_signal") ;
    ros::Node::instance()->unadvertise("full_cloud") ;
  }

  void scannerSignalCallback()
  {
    if (cur_signal_.signal == 128 || cur_signal_.signal == 129)       // These codes imply that this is the first signal during a given profile type
      first_time_ = true ;


    if (first_time_)
    {
      prev_signal_ = cur_signal_ ;
      first_time_ = false ;
    }
    else
    {
      if(num_skips_>0)
      {
        if(num_skips_left_>0)
        {
          num_skips_left_ -= 1 ;
          return;
        }
        else
        {
          num_skips_left_=num_skips_;
        }
      }

      BuildCloud::Request req ;
      BuildCloud::Response resp ;

      req.begin = prev_signal_.header.stamp ;
      req.end   = cur_signal_.header.stamp ;

      if (!ros::service::call("build_cloud", req, resp))
	ROS_ERROR("Failed to call service on point cloud assembler or laser scan assembler.");

      ros::Node::instance()->publish("full_cloud", resp.cloud) ;
      ROS_DEBUG("Snapshotter::Published Cloud size=%u", resp.cloud.get_points_size()) ;

      prev_signal_ = cur_signal_ ;
    }
  }
} ;

}

using namespace point_cloud_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  PointCloudSnapshotter snapshotter ;
  ros::Node::instance()->spin();
  
  return 0;
}
