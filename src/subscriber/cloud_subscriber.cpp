/*
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:27:59
 */

#include "subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"
#include "pcl_conversions/pcl_conversions.h"

namespace avp_mapping {

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string topic_name,
                                 size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(
    const sensor_msgs::PointCloud2ConstPtr &cloud_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  CloudData cloud_data;
  cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
  pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));
  latest_deque_cloud_data_.push_back(cloud_data);
}

void CloudSubscriber::parseData(std::deque<CloudData> &deque_cloud_data_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_deque_cloud_data_.size() > 0) {
    deque_cloud_data_buff.insert(deque_cloud_data_buff.end(),
                                 latest_deque_cloud_data_.begin(),
                                 latest_deque_cloud_data_.end());
    latest_deque_cloud_data_.clear();
  }
}

} // namespace avp_mapping
