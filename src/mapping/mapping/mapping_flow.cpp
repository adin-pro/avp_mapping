/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:37:37
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 21:14:30
 */

#include "mapping/mapping/mapping_flow.hpp"

namespace avp_mapping {
MappingFlow::MappingFlow(ros::NodeHandle &nh) {
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/bev/rgb_cloud", 1000);
  odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/odom", 10000);
  cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/map_cloud", "/map", 1000);

}

bool MappingFlow::run() {
  if (!readData())
    return false;
  while (hasData()) {
    if (validData()) {
      continue;
    }
    publishData();
  }
  return true;
}

bool MappingFlow::readData() {
  cloud_sub_ptr_->parseData(cloud_data_buff_);
  odom_sub_ptr_->parseData(odom_data_buff_);
  return true;
}

bool MappingFlow::hasData() {
  if (cloud_data_buff_.size() == 0)
    return false;
  if (odom_data_buff_.size() == 0)
    return false;
  return true;
}

bool MappingFlow::validData() {
  cuur_cloud_data_ = cloud_data_buff_.front();
  curr_odom_ = odom_data_buff_.front();
  double time_diff = curr_odom_.time - cuur_cloud_data_.time;
  if (time_diff < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (time_diff > 0.05) {
    odom_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  odom_data_buff_.pop_front();
  return true;
}

bool MappingFlow::publishData() { return true; }

} // namespace avp_mapping