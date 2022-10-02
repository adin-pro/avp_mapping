/*
 * @Author: ding.yin
 * @Date: 2022-10-02 21:50:48
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:55:31
 */
#ifndef _IMU_SUBSCRIBER_H_
#define _IMU_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "sensor_data/imu_data.hpp"

namespace avp_mapping {

class IMUSubscriber {

public:
  IMUSubscriber() = default;
  IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
  void parseData(std::deque<IMUData> &deque_imu_data);

private:
  void msg_callback(const sensor_msgs::Imu::ConstPtr &imu_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<IMUData> latest_deque_imu_data_;
  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _IMU_SUBSCRIBER_H_