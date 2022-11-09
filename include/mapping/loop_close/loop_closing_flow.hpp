/*
 * @Author: ding.yin
 * @Date: 2022-11-09 19:42:09
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 20:38:34
 */

#ifndef _LOOP_CLOSING_FLOW_H_
#define _LOOP_CLOSING_FLOW_H_

#include "mapping/loop_close/loop_closing.hpp"

#include "ros/ros.h"

#include "publisher/loop_pose_publisher.hpp"
#include "subscriber/key_frame_subscriber.hpp"

namespace avp_mapping {
class LoopClosingFlow {
public:
  LoopClosingFlow(ros::NodeHandle &nh, std::string work_dir);

  bool run();

private:
  bool readData();
  bool hasData();
  bool validData();
  bool publishData();

private:
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> reli_odom_sub_ptr_;

  std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;

  std::shared_ptr<LoopClosing> loop_closing_ptr_;

  std::deque<KeyFrame> key_frame_buff_;
  std::deque<KeyFrame> reli_odom_buff_;

  KeyFrame curr_key_frame_;
  KeyFrame curr_reli_odom_;
};

} // namespace avp_mapping

#endif // _LOOP_CLOSING_FLOW_H_