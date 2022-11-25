/*
 * @Author: ding.yin
 * @Date: 2022-11-24 13:38:00
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-24 22:04:51
 */

#ifndef _SEMANTIC_LOOP_CLOSING_H_
#define _SEMANTIC_LOOP_CLOSING_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>

#include "sensor_data/cloud_data.hpp"
#include "sensor_data/image_data.hpp"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/loop_pose.hpp"

#include "global_defination/avp_labels.hpp"
#include "models/loop_close/semantic_list.hpp"
#include "models/loop_close/semantic_node.hpp"

namespace avp_mapping {
class SemanticLoopClosing {
public:
  SemanticLoopClosing(YAML::Node &node);

  SemanticList CreateSListFromImage(const ImageData &image_data,
                                    const KeyFrame &kf);
  bool InsertSList(const SemanticList &sList, CloudData &cloud_height);
  bool HasEnoughObjects(const SemanticList &sList);
  bool FindBestMatch(const SemanticList &query, SemanticList &responce);
  bool CalcRelativePoseToTarget(SemanticList &source, SemanticList &target,
                                Eigen::Matrix4d &trans_src_to_tar);
  bool TryGetLoopPose(const ImageData &image_data, CloudData &cloud_height,
                      LoopPose &lp, const KeyFrame &kf);

private:
  void find_min_rects_crossing(cv::Mat img,
                               std::vector<std::vector<double>> &out_rects,
                               double min_area);
  void find_min_rects(cv::Mat &img, std::vector<std::vector<double>> &out_rects,
                      double min_area);

private:
  std::vector<SemanticList> sList_database_;
  std::unordered_map<int, CloudData> kf_cloud_height_database_;

  int loop_pose_step_ = 5;
  int start_up_index_ = 80;
  int last_loop_pose_index = 0;
  int index_exclude_range_ = 5;
  double coarse_similarity_limit_ = 0.95;
  double fine_similarity_limit_ = 5.0;
  int coarse_candidates_ = 10;

  // piror object size
  std::vector<double> prior_size_ = std::vector<double>(AVPColors.size(), 0.0);
  // coeffs for claculating similarity
  std::vector<double> coarse_semantic_coeff_ =
      std::vector<double>(AVPColors.size(), 0.0);
  std::vector<double> fine_semantic_coeff_ =
      std::vector<double>(AVPColors.size(), 0.0);

  pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT> icp;

  double fitness_upper_bound_ = 2.0;
  int max_iteration_ = 20;
};

} // namespace avp_mapping

#endif // _SEMANTIC_LOOP_CLOSING_H_
