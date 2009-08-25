/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/node.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_scan/median_filter.h"
#include "laser_scan/intensity_filter.h"
#include "laser_scan/scan_shadows_filter.h"
#include "laser_scan/footprint_filter.h"
#include "tf/message_notifier.h"


class GenericLaserScanFilterNode 
{
public:
  GenericLaserScanFilterNode(ros::Node& anode) :  filter_chain_("sensor_msgs::LaserScan"), node_(anode), notifier_(NULL)
  {
    node_.advertise<sensor_msgs::LaserScan>("~output", 1000);
    notifier_ = new tf::MessageNotifier<sensor_msgs::LaserScan>(&tf_, &node_, 
        boost::bind(&GenericLaserScanFilterNode::callback, this, _1), "scan_in", "base_link", 50);
    notifier_->setTolerance(ros::Duration(0.03));

    filter_chain_.configure("~");
    //node_.subscribe("scan_in", msg, &GenericLaserScanFilterNode::callback,this, 3);
  }

  ~GenericLaserScanFilterNode(){
    if(notifier_)
      delete notifier_;
  }

  void callback(const tf::MessageNotifier<sensor_msgs::LaserScan>::MessagePtr& msg_in)
  {
    filter_chain_.update (*msg_in, msg);
    node_.publish("~output", msg);
  }

protected:
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  ros::Node& node_;
  tf::TransformListener tf_;
  tf::MessageNotifier<sensor_msgs::LaserScan>* notifier_;
  sensor_msgs::LaserScan msg;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node ros_node("scan_filter_node");
  
  GenericLaserScanFilterNode t(ros_node);
  ros_node.spin();
  
  return 0;
}

