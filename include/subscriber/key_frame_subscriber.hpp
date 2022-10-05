/*
 * @Author: ding.yin
 * @Date: 2022-10-05 16:31:27
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 16:36:45
 */

#ifndef _KEY_FRAME_SUBSCRIBER_H_
#define _KEY_FRAME_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

#include "sensor_data/key_frame.hpp"

namespace avp_mapping {
class KeyFrameSubscriber {

public:
  KeyFrameSubscriber(ros::NodeHandle &nh, std::string topic_name,
                     size_t buff_size);
  KeyFrameSubscriber() = default;
  void parseData(std::deque<KeyFrame> &deque_key_frame);

private:
  void msg_callback(
      const geometry_msgs::PoseWithCovarianceConstPtr &key_frame_msg_ptr);

private:
  std::deque<KeyFrame> latest_deque_key_frame_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _KEY_FRAME_SUBSCRIBER_H_
