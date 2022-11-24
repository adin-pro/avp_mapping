/*
 * @Author: ding.yin
 * @Date: 2022-11-09 19:28:48
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 20:31:12
 */
#ifndef _LOOP_CLOSING_H_
#define _LOOP_CLOSING_H_

#include <deque>

#include "yaml-cpp/yaml.h"

#include "models/cloud_filter/cloud_filter_interface.hpp"
#include "models/registration/registration_interface.hpp"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/loop_pose.hpp"

namespace avp_mapping {
class LoopClosing {
public:
  LoopClosing(std::string work_dir);
  bool update(const KeyFrame key_frame, const KeyFrame key_reli_odom);
  bool hasNewLoopPose();
  LoopPose &getCurrentLoopPose();

private:
  bool initWithConfig(std::string work_dir);
  bool initParam(const YAML::Node &node);
  bool initDataPath(const YAML::Node &node);
  bool initRegistration(std::shared_ptr<RegistrationInterface> &reg_ptr,
                        const YAML::Node &node);
  bool initFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface> &filter_ptr,
                  const YAML::Node &node);

  bool detectNearestKeyFrame(int &key_frame_index);
  bool cloudRegistration(int key_frame_index);
  bool jointMap(int key_frame_index, CloudData::CLOUD_PTR &map_cloud_ptr,
                Eigen::Matrix4d &map_pose);
  bool jointScan(CloudData::CLOUD_PTR &scan_cloud_ptr,
                 Eigen::Matrix4d &scan_pose);
  bool registration(CloudData::CLOUD_PTR &map_cloud_ptr,
                    CloudData::CLOUD_PTR &scan_cloud_ptr,
                    Eigen::Matrix4d &scan_pose, Eigen::Matrix4d &result_pose);

private:
  std::string key_frame_path_ = "";
  int extend_frame_num_ = 3;
  int loop_step_ = 10;
  int diff_num_ = 100;
  double detect_area_ = 10.0;
  double fitness_score_limit_ = 2.0;

  std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;

  std::deque<KeyFrame> all_key_frames_;
  std::deque<KeyFrame> all_key_reli_odoms_;

  LoopPose curr_loop_pose_;
  bool has_new_loop_pose_ = false;
};

} // namespace avp_mapping

#endif // _LOOP_CLOSING_H_