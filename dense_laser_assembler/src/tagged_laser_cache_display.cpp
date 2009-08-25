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

#include <string>
#include <vector>

#include "ros/ros.h"

#include "message_filters/msg_cache.h"
#include "dense_laser_assembler/joint_extractor.h"
#include "dense_laser_assembler/laser_scan_tagger.h"

using namespace std ;
using namespace dense_laser_assembler ;
using namespace message_filters ;

typedef LaserScanTagger<JointExtractor::JointArray> LaserTagger ;

void display_tagged_scan(const boost::shared_ptr<const LaserTagger::TaggedLaserScan>& tagged_scan )
{
  printf("TaggedScan:\n") ;
  printf("  Laser Scan Time:   %f\n", tagged_scan->scan->header.stamp.toSec()) ;
  printf("  JointArray before: %f\n", tagged_scan->before->header.stamp.toSec()) ;
  printf("  JointArray after:  %f\n", tagged_scan->after->header.stamp.toSec()) ;
  printf("  Joints:\n") ;

  for (unsigned int i=0; i< tagged_scan->before->positions.size(); i++)
  {
    printf("    %02u) %f->%f\n", i, tagged_scan->before->positions[i], tagged_scan->after->positions[i]) ;
  }
  printf("*****\n") ;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tagged_laser_cache_display") ;

  ros::NodeHandle nh ;

  // Define which links we want to listen to
  vector<string> joint_names ;
  joint_names.push_back("laser_tilt_mount_joint") ;
  joint_names.push_back("torso_lift_link") ;

  // Set up the joint extraction and cache
  JointExtractor joint_extractor(joint_names) ;
  MsgCache<JointExtractor::JointArray> joint_cache(100) ;
  joint_cache.subscribe(joint_extractor) ;

  // Set up the laser tagger (and link it to the joint_cache)
  LaserTagger laser_tagger(joint_cache, 40) ;
  joint_cache.addOutputCallback(boost::bind(&LaserTagger::update, &laser_tagger)) ;
  laser_tagger.addOutputCallback(&display_tagged_scan) ;

  // Set up the cache for tagged laser scans (and link it to the laser_tagger)
  MsgCache<LaserTagger::TaggedLaserScan> tagged_laser_cache(40) ;
  tagged_laser_cache.subscribe(laser_tagger) ;


  // Link the joint extractor to a topic
  ros::Subscriber mech_sub  = nh.subscribe("mechanism_state", 1, &JointExtractor::processMechState, &joint_extractor) ;

  // Link the laser_tagger to a topic
  ros::Subscriber laser_sub = nh.subscribe("tilt_scan", 1, &LaserTagger::processLaserScan, &laser_tagger) ;

  ros::spin() ;


  return 0 ;
}
