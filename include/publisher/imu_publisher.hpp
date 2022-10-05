/*
 * @Author: ding.yin
 * @Date: 2022-10-05 19:11:24
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 19:35:56
 */
#ifndef _IMU_PUBLISHER_H_
#define _IMU_PUBLISHER_H_

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "sensor_data/imu_data.hpp"

namespace avp_mapping {
class IMUPublisher {
public:
  IMUPublisher() = default;
  IMUPublisher(ros::NodeHandle &nh_, std::string topic_name,
               std::string frame_id, size_t buff_size);

  void publish(IMUData imu_input);

  bool hasSubscriber();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _IMU_PUBLISHER_H_