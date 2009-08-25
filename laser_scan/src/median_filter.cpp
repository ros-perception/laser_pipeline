#include "laser_scan/median_filter.h"

namespace laser_scan
{
LaserMedianFilter::LaserMedianFilter():
  num_ranges_(1), xmlrpc_value_()
{
  
};

bool LaserMedianFilter::configure()
{
  
  if (!getParam("internal_filter", xmlrpc_value_))
  {
    ROS_ERROR("Cannot Configure LaserMedianFilter: Didn't find \"internal_filter\" tag within LaserMedianFilter params. Filter definitions needed inside for processing range and intensity");
    return false;
  }
  
  if (range_filter_) delete range_filter_;
  range_filter_ = new filters::MultiChannelFilterChain<float>("float");
  if (!range_filter_->configure(num_ranges_, xmlrpc_value_)) return false;
  
  if (intensity_filter_) delete intensity_filter_;
  intensity_filter_ = new filters::MultiChannelFilterChain<float>("float");
  if (!intensity_filter_->configure(num_ranges_, xmlrpc_value_)) return false;
  return true;
};

LaserMedianFilter::~LaserMedianFilter()
{
  delete range_filter_;
  delete intensity_filter_;
};

bool LaserMedianFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
  if (!this->configured_) 
  {
    ROS_ERROR("LaserMedianFilter not configured");
    return false;
  }
  boost::mutex::scoped_lock lock(data_lock);
  scan_out = scan_in; ///Quickly pass through all data \todo don't copy data too


  if (scan_in.get_ranges_size() != num_ranges_) //Reallocating
  {
    ROS_INFO("Laser filter clearning and reallocating due to larger scan size");
    delete range_filter_;
    delete intensity_filter_;


    num_ranges_ = scan_in.get_ranges_size();
    
    range_filter_ = new filters::MultiChannelFilterChain<float>("float");
    if (!range_filter_->configure(num_ranges_, xmlrpc_value_)) return false;
    
    intensity_filter_ = new filters::MultiChannelFilterChain<float>("float");
    if (!intensity_filter_->configure(num_ranges_, xmlrpc_value_)) return false;
    
  }

  /** \todo check for length of intensities too */
  range_filter_->update(scan_in.ranges, scan_out.ranges);
  intensity_filter_->update(scan_in.intensities, scan_out.intensities);


  return true;
}
}
