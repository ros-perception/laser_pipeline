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

#include "dense_laser_assembler/build_dense_laser_snapshot.h"

using namespace std ;
using namespace boost ;

namespace dense_laser_assembler
{

bool verifyMetadata(const calibration_msgs::DenseLaserSnapshot& snapshot, const sensor_msgs::LaserScan& scan) ;

bool buildDenseLaserSnapshot(const vector<shared_ptr<const JointTaggedLaserScan> >& scans,
                             const vector<string>& joint_names,
                             calibration_msgs::DenseLaserSnapshot& snapshot)
{
  if (scans.size() == 0)
  {
    ROS_WARN("Trying to build empty cloud") ;
    snapshot.angle_min       = 0.0 ;
    snapshot.angle_max       = 0.0 ;
    snapshot.angle_increment = 0.0 ;
    snapshot.time_increment  = 0.0 ;
    snapshot.range_min       = 0.0 ;
    snapshot.range_max       = 0.0 ;
    snapshot.readings_per_scan = 0 ;
    snapshot.num_scans       = 0 ;
    snapshot.ranges.clear() ;
    snapshot.intensities.clear() ;
    snapshot.joint_encoding = 0 ;
    snapshot.joint_names.clear() ;
    snapshot.joint_positions.clear() ;
    snapshot.joint_velocities.clear() ;
    snapshot.scan_start.clear() ;
    return true ;
  }

  snapshot.header.stamp = scans[scans.size()-1]->scan->header.stamp ;

  // Fill in all the metadata
  snapshot.header.frame_id = scans[0]->scan->header.frame_id ;
  snapshot.angle_min       = scans[0]->scan->angle_min ;
  snapshot.angle_max       = scans[0]->scan->angle_max ;
  snapshot.angle_increment = scans[0]->scan->angle_increment ;
  snapshot.time_increment  = scans[0]->scan->time_increment ;
  snapshot.range_min       = scans[0]->scan->range_min ;
  snapshot.range_max       = scans[0]->scan->range_max ;

  // Define the data dimensions
  snapshot.readings_per_scan = scans[0]->scan->ranges.size() ;
  snapshot.num_scans      = scans.size() ;
  const unsigned int& w = snapshot.readings_per_scan ;
  const unsigned int& h = snapshot.num_scans ;

  // Do a consistency check on the metadata
  for (unsigned int i=0; i<scans.size(); i++)
  {
    if (!verifyMetadata(snapshot, *scans[i]->scan))
    {
      ROS_WARN("Metadata doesn't match") ;
      return false ;
    }
  }

  // Set up joint data vectors
  const unsigned int num_joints = joint_names.size() ;
  snapshot.joint_names = joint_names ;
  snapshot.joint_encoding = calibration_msgs::DenseLaserSnapshot::POS_PER_ROW_START_AND_END ;
  snapshot.joint_positions.resize(num_joints*h*2) ; // 2 Positions per joint per row (beginning and end of row)
  snapshot.joint_velocities.resize(num_joints*h*2) ;

  // Set up other data vectors
  snapshot.scan_start.resize(h) ;
  snapshot.ranges.resize(w*h) ;
  snapshot.intensities.resize(w*h) ;

  const unsigned int range_elem_size     = sizeof(scans[0]->scan->ranges[0]) ;
  const unsigned int intensity_elem_size = sizeof(scans[0]->scan->intensities[0]) ;

  for (unsigned int i=0; i<h; i++)
  {
    memcpy(&snapshot.ranges[i*w],      &scans[i]->scan->ranges[0],      w*range_elem_size) ;
    memcpy(&snapshot.intensities[i*w], &scans[i]->scan->intensities[0], w*intensity_elem_size) ;

    // Copy joint positions & velocities
    for (unsigned int j=0; j<num_joints; j++)
    {
      snapshot.joint_positions[i*j*2 + 0] = scans[i]->before->pos[j] ;
      snapshot.joint_positions[i*j*2 + 1] = scans[i]->after->pos[j] ;
      snapshot.joint_velocities[i*j*2 + 0] = scans[i]->before->vel[j] ;
      snapshot.joint_velocities[i*j*2 + 1] = scans[i]->after->vel[j] ;
    }

    // Copy time stamp
    snapshot.scan_start[i] = scans[i]->header.stamp ;
  }

  ROS_DEBUG("Done building snapshot that is [%u rows] x [%u cols]", h, w) ;

  return true ;
}

static const double eps = 1e-9 ;

#define CHECK(a) \
{ \
  if ( (snapshot.a - scan.a < -eps) || (snapshot.a - scan.a > eps)) \
    return false ; \
}

bool verifyMetadata(const calibration_msgs::DenseLaserSnapshot& snapshot, const sensor_msgs::LaserScan& scan)
{
  CHECK(angle_min) ;
  CHECK(angle_max) ;
  CHECK(angle_increment) ;
  CHECK(time_increment) ;
  CHECK(range_min) ;
  CHECK(range_max) ;

  if (snapshot.header.frame_id.compare(scan.header.frame_id) != 0)
    return false ;

  if (snapshot.readings_per_scan != scan.ranges.size())
    return false ;
  if (snapshot.readings_per_scan != scan.intensities.size())
    return false ;


  return true ;
}

}
