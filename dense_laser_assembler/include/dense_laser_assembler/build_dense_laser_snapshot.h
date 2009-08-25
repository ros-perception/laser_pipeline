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

#ifndef DENSE_LASER_ASSEMBLER_BUILD_DENSE_LASER_SNAPSHOT_H_
#define DENSE_LASER_ASSEMBLER_BUILD_DENSE_LASER_SNAPSHOT_H_


#include <vector>
#include "dense_laser_assembler/laser_scan_tagger.h"
#include "message_filters/cache.h"

#include "dense_laser_assembler/JointPVArray.h"
#include "dense_laser_assembler/tagged_laser_scan.h"

#include "calibration_msgs/DenseLaserSnapshot.h"


namespace dense_laser_assembler
{

typedef LaserScanTagger<JointPVArray> LaserJointTagger ;
typedef TaggedLaserScan<JointPVArray> JointTaggedLaserScan ;

/**
 * \brief Builds a dense laser snapshot
 * Grabs all the scans between the start and end time, and composes them into one larger snapshot. This call is non-blocking.
 * Thus, if there are scans that haven't been cached yet, but occur before the end time, they won't be added to the snapshot.
 * \param start The earliest scan time to be included in the snapshot
 * \param end The latest scan time to be included in the snapshot
 * \param output: A populated snapshot message
 */

bool buildDenseLaserSnapshot(const std::vector<boost::shared_ptr<const JointTaggedLaserScan> >& scans,
                             const std::vector<std::string>& joint_names,
                             calibration_msgs::DenseLaserSnapshot& snapshot) ;


}


#endif // DENSE_LASER_ASSEMBLER_BUILD_DENSE_LASER_SNAPSHOT
