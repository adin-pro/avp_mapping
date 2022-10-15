/*
 * @Author: ding.yin
 * @Date: 2022-10-10 11:28:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 15:21:37
 */

#include "subscriber/velocity_subscriber.hpp"

namespace avp_mapping {

VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh,
                                       std::string topic_name, size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::msg_callback(
    const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  VelocityData velo;
  velo.time = twist_msg_ptr->header.stamp.toSec();
  velo.angular_velocity.x = twist_msg_ptr->twist.angular.x;
  velo.angular_velocity.y = twist_msg_ptr->twist.angular.y;
  velo.angular_velocity.z = twist_msg_ptr->twist.angular.z;

  velo.linear_velocity.x = twist_msg_ptr->twist.linear.x;
  velo.linear_velocity.y = twist_msg_ptr->twist.linear.y;
  velo.linear_velocity.z = twist_msg_ptr->twist.linear.z;

  latest_velocity_data_.push_back(velo);
}

void VelocitySubscriber::parseData(std::deque<VelocityData> &velo_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_velocity_data_.size() > 0) {
    velo_buff.insert(velo_buff.end(), latest_velocity_data_.begin(),
                     latest_velocity_data_.end());
    latest_velocity_data_.clear();
  }
}

} // namespace avp_mapping