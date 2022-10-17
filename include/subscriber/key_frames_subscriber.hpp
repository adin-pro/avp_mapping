/*
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:41:57
 */

#ifndef _KEY_FRAMES_SUBSCRIBER_H_
#define _KEY_FRAMES_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <sensor_data/key_frame.hpp>

namespace avp_mapping {
class KeyFramesSubscriber {

public:
  KeyFramesSubscriber() = default;
  KeyFramesSubscriber(ros::NodeHandle &nh, std::string topic_name,
                      size_t buff_size);
  void parseData(std::deque<KeyFrame>& deque_key_frames);

private:
  void msg_callback(const nav_msgs::Path::ConstPtr &key_frames_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<KeyFrame> latest_deque_key_frames_;
  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _KEY_FRAMES_SUBSCRIBER_H_