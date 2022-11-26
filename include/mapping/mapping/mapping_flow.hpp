/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:57:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 17:47:33
 */

#ifndef _MAPPING_FLOW_H_
#define _MAPPING_FLOW_H_

#include <deque>

#include "ros/ros.h"

#include "mapping/mapping/mapping_core.hpp"
#include "publisher/cloud_publisher.hpp"
#include "sensor_data/key_frame.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/key_frames_subscriber.hpp"
#include "subscriber/key_frame_subscriber.hpp"

namespace avp_mapping {
class MappingFlow {
public:
  MappingFlow(ros::NodeHandle &nh, const std::string work_dir);

  bool run();

private:
  bool readData();
  bool validData();
  bool hasData();
  bool transformData();
  bool publishData();

private:
  std::deque<CloudData> kf_cloud_data_buff_;
  std::deque<KeyFrame> back_end_kfs_data_buff_;
  std::deque<KeyFrame> front_end_kf_data_buff_;

  std::shared_ptr<CloudSubscriber> kf_cloud_sub_ptr_;
  std::shared_ptr<KeyFramesSubscriber> back_end_kfs_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> front_end_kf_sub_ptr_;
  std::shared_ptr<CloudPublisher> optimized_cloud_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> original_cloud_map_pub_ptr_;

  std::shared_ptr<MappingCore> mapping_core_ptr_;

  CloudData optimized_cloud_map_;
  CloudData original_cloud_map_;
  bool save_map_ = false;
  std::string save_dir_ = "./";
};

} // namespace avp_mapping

#endif // _MAPPING_FLOW_H_
