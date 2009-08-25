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

#ifndef DENSE_LASER_ASSEMBLER_LASER_SCAN_TAGGER_H_
#define DENSE_LASER_ASSEMBLER_LASER_SCAN_TAGGER_H_

#include <deque>
#include "sensor_msgs/LaserScan.h"
#include "message_filters/cache.h"
#include "boost/shared_ptr.hpp"
#include "dense_laser_assembler/tagged_laser_scan.h"

#include <message_filters/simple_filter.h>

namespace dense_laser_assembler
{

/**
 * Listens to LaserScans and a cache of 'tag' messages. Outputs a laserScan
 * whenever there is a tag msg that occurs before and after the scan.
 */
template <class T>
class LaserScanTagger : public message_filters::SimpleFilter<TaggedLaserScan<T> >
{
public:

  typedef boost::shared_ptr<const T> TConstPtr ;
  typedef boost::shared_ptr<const TaggedLaserScan<T> > MConstPtr ;



  template<class A>
  /**
   * \brief Construct object, and also subscribe to relevant data sources
   */
  LaserScanTagger(A& a, message_filters::Cache<T>& tag_cache, unsigned int max_queue_size)
  {
    subscribeLaserScan(a);
    subscribeTagCache(tag_cache);
    setMaxQueueSize(max_queue_size);
    tag_cache_ = &tag_cache ;
    max_queue_size_ = max_queue_size ;
  }

  LaserScanTagger()
  {
    tag_cache_ = NULL;
    setMaxQueueSize(1);
  }

  ~LaserScanTagger()
  {
    incoming_laser_scan_connection_.disconnect() ;
    incoming_tag_connection_.disconnect() ;
  }

  template<class A>
  void subscribeLaserScan(A& a)
  {
    incoming_laser_scan_connection_ = a.registerCallback(boost::bind(&LaserScanTagger<T>::processLaserScan, this, _1));
  }

  void subscribeTagCache(message_filters::Cache<T>& tag_cache)
  {
    tag_cache_ = &tag_cache ;
    incoming_tag_connection_ = tag_cache.registerCallback(boost::bind(&LaserScanTagger<T>::processTag, this, _1));
  }

  void setMaxQueueSize(unsigned int max_queue_size)
  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    max_queue_size_ = max_queue_size;
  }

  /**
   * Adds the laser scan onto the queue of scans that need to be matched with the tags
   */
  void processLaserScan(const sensor_msgs::LaserScanConstPtr& msg)
  {
    {
      boost::mutex::scoped_lock lock(queue_mutex_);
      //! \todo need to more carefully decide on overflow logic
      if (queue_.size() < max_queue_size_)
        queue_.push_back(msg) ;
      else
        ROS_WARN("Queue full, not pushing new data onto queue until queue is serviced") ;
    }

    update() ;
  }


  /**
   * Since we just received a new tag message, we might have laser scans that we need to process
   */
  void processTag(const TConstPtr& msg)
  {
    update() ;
  }

  /**
   * Triggers the module to try to service the queue
   */
  void update()
  {
    //! \todo This is not a good enough check. We need to somehow make sure the cache hasn't been destructed
    if (!tag_cache_)
    {
      ROS_WARN("Have a NULL pointer to TagCache. Skipping update");
      return;
    }

    bool did_something = true ;

    // It's possible we need to process multiple laser scans. Therefore, keep
    // looping until we reach a scan that's a no-op
    while (true)
    {
      //! \todo Come up with better locking/unlocking in this loop
      queue_mutex_.lock();

      // Exit condition
      if (!did_something || queue_.size() == 0)
      {
        queue_mutex_.unlock();
        break;
      }

      did_something = false ;   // Haven't done anything yet

      const sensor_msgs::LaserScanConstPtr& elem = queue_.front() ;

      ros::Time scan_start = elem->header.stamp ;
      ros::Time scan_end   = elem->header.stamp + ros::Duration().fromSec(elem->time_increment*elem->ranges.size()) ;

      boost::shared_ptr<const T> tag_before = tag_cache_->getElemBeforeTime(scan_start) ;
      boost::shared_ptr<const T> tag_after  = tag_cache_->getElemAfterTime(scan_end) ;


      if (!tag_before)          // Can't get old enough tag. Give up
      {
        queue_.pop_front() ;
        did_something = true ;
        queue_mutex_.unlock() ;
      }
      else if (!tag_after)      // Don't have new enough tag. Keep scan on queue until we get it
      {
        did_something = false ;
        queue_mutex_.unlock() ;
      }
      else                      // Both tags are fine. Use the data, and push out
      {
        did_something = true;
        boost::shared_ptr<TaggedLaserScan<T> > tagged_scan(new TaggedLaserScan<T> ) ;
        tagged_scan->header = elem->header ;
        tagged_scan->scan   = elem ;
        tagged_scan->before = tag_before ;
        tagged_scan->after  = tag_after ;
        queue_.pop_front() ;
        queue_mutex_.unlock() ;

        signalMessage(tagged_scan) ;
      }
    }
  }

private:

  std::deque<sensor_msgs::LaserScanConstPtr> queue_ ;    //!< Incoming queue of laser scans
  boost::mutex queue_mutex_ ;                            //!< Mutex for laser scan queue
  unsigned int max_queue_size_ ;                         //!< Max # of laser scans to queue up for processing
  message_filters::Cache<T>* tag_cache_ ;             //!< Cache of the tags that we need to merge with laser data

  message_filters::Connection incoming_laser_scan_connection_;
  message_filters::Connection incoming_tag_connection_;
} ;

}



#endif /* DENSE_LASER_ASSEMBLER_LASER_SCAN_TAGGER_H_ */
