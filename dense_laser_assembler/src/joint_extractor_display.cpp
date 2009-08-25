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

#include <boost/signals.hpp>

#include "ros/ros.h"
#include "dense_laser_assembler/joint_pv_msg_filter.h"
//#include "dense_laser_assembler/joint_pv_diag_msg_filter.h"
#include "dense_laser_assembler/JointPVArray.h"

#include "diagnostic_updater/diagnostic_updater.h"

using namespace std ;
//using namespace message_filters ;
using namespace dense_laser_assembler ;

void display_joints(vector<string> joint_names, const JointPVArrayConstPtr& joint_array_ptr)
{
  printf("Joints:\n") ;
  for (unsigned int i=0; i< joint_names.size(); i++)
  {
    printf("  %s: %f, %f\n", joint_names[i].c_str(), joint_array_ptr->pos[i], joint_array_ptr->vel[i]) ;
  }

}

void diagnosticsLoop(diagnostic_updater::Updater* diagnostic)
{
  ros::NodeHandle nh ;
  while(nh.ok())
  {
    diagnostic->update() ;
    sleep(1) ;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_extractor_display") ;

  ros::NodeHandle nh ;

  vector<string> joint_names ;
  joint_names.push_back("laser_tilt_mount_joint") ;
  joint_names.push_back("torso_lift_link") ;

  diagnostic_updater::Updater diagnostic(nh) ;

  JointPVMsgFilter joint_extractor(joint_names) ;
  //JointPVDiagMsgFilter joint_extractor(diagnostic, joint_names) ;

  joint_extractor.registerCallback( boost::bind(&display_joints, joint_names, _1) ) ;

  ros::Subscriber sub = nh.subscribe("mechanism_state", 1, &JointPVMsgFilter::processMechState, &joint_extractor) ;

  //boost::thread* diagnostic_thread ;
  //diagnostic_thread = new boost::thread( boost::bind(&diagnosticsLoop, &diagnostic) );

  ros::spin() ;

  return 0 ;
}
