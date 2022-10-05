/*
 * @Author: ding.yin
 * @Date: 2022-10-05 17:16:21
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 17:21:21
 */

#ifndef _VELOCITY_SUBSCRIBER_H_
#define _VELOCITY_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

#include "sensor_data/velocity_data.hpp"

namespace avp_mapping {

class VelocitySubscriber {

public:
  VelocitySubscriber() = default;
  VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name,
                     size_t buff_size);
  void parseData(std::deque<VelocityData> &deque_velocity_data);

private:
  void msg_callback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<VelocityData> latest_velocity_data_;

  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _VELOCITY_SUBSCRIBER_H_