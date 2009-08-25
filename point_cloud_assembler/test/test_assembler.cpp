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

/* Author: Vijay Pradeep */

#include <string>
#include <gtest/gtest.h>
#include "ros/node.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "boost/thread.hpp"

using namespace ros;
using namespace sensor_msgs;

int g_argc;
char** g_argv;

class TestAssembler : public testing::Test
{
public:

  NodeHandle n_;

  sensor_msgs::PointCloud cloud_msg_ ;
  boost::mutex cloud_mutex_ ;
  sensor_msgs::PointCloud safe_cloud_ ;
  int cloud_counter_ ;

  LaserScan scan_msg_ ;
  boost::mutex scan_mutex_ ;
  LaserScan safe_scan_ ;
  int scan_counter_ ;


  void CloudCallback(const PointCloudConstPtr& cloud_msg)
  {
    cloud_mutex_.lock() ;
    cloud_counter_++ ;
    safe_cloud_ = *cloud_msg ;
    cloud_mutex_.unlock() ;
    ROS_INFO("Got Cloud with %u points", cloud_msg_.get_points_size()) ;
  }

  void ScanCallback(const LaserScanConstPtr& scan_msg)
  {
    scan_mutex_.lock() ;
    scan_counter_++ ;
    safe_scan_ = *scan_msg ;
    scan_mutex_.unlock() ;
    //ROS_INFO("Got Scan") ;
  }

protected:
  /// constructor
  TestAssembler()
  {
    cloud_counter_ = 0 ;
    scan_counter_ = 0 ;
  }

  /// Destructor
  ~TestAssembler()
  {

  }
};


TEST_F(TestAssembler, test)
{
  ROS_INFO("Starting test F");
  ros::Subscriber cloud_sub = n_.subscribe("dummy_cloud", 10, &TestAssembler::CloudCallback, (TestAssembler*)this);
  ros::Subscriber scan_sub  = n_.subscribe("dummy_scan",  10, &TestAssembler::ScanCallback,  (TestAssembler*)this);

  bool waiting = true;
  while(n_.ok() && waiting )
  {
    ros::spinOnce();
    usleep(1e2) ;

    scan_mutex_.lock() ;
    waiting = (scan_counter_ < 55) ;
    scan_mutex_.unlock() ;
  }

  usleep(1e6) ;

  ASSERT_GT(cloud_counter_, 0) ;

  unsigned int cloud_size ;
  cloud_mutex_.lock() ;
  cloud_size = safe_cloud_.get_points_size() ;
  cloud_mutex_.unlock() ;

  EXPECT_EQ((unsigned int) 1000, cloud_size) ;

  SUCCEED();
}

int main(int argc, char** argv)
{
  printf("******* Starting application *********\n");

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_assembler_node");

  return RUN_ALL_TESTS();
}
