/*
 * @Author: ding.yin
 * @Date: 2022-10-08 19:02:20
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-08 19:18:46
 */

#include "subscriber/imu_subscriber.hpp"

namespace avp_mapping {

IMUSubscriber::IMUSubscriber(ros::NodeHandle &nh, std::string topic_name,
                             size_t buff_size)
    : nh_(nh) {
  subscriber_ =
      nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr &imu_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  IMUData imu_data;
  imu_data.time = imu_msg_ptr->header.stamp.toSec();
  imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
  imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
  imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

  imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
  imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
  imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

  imu_data.orientation.x = imu_msg_ptr->orientation.x;
  imu_data.orientation.y = imu_msg_ptr->orientation.y;
  imu_data.orientation.z = imu_msg_ptr->orientation.z;
  imu_data.orientation.w = imu_msg_ptr->orientation.w;

  latest_deque_imu_data_.push_back(imu_data);
}

void IMUSubscriber::parseData(std::deque<IMUData> &imu_data_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_deque_imu_data_.size() > 0) {
    imu_data_buff.insert(imu_data_buff.end(), latest_deque_imu_data_.begin(),
                         latest_deque_imu_data_.end());
    latest_deque_imu_data_.clear();
  }
}

} // namespace avp_mapping
