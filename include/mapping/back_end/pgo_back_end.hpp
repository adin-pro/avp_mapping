/*
 * @Author: ding.yin
 * @Date: 2022-11-25 14:02:55
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 15:32:58
 */

#ifndef _PGO_BACK_END_H_
#define _PGO_BACK_END_H_

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include "sensor_data/key_frame.hpp"
#include "sensor_data/loop_pose.hpp"

#include "models/optimizer/ceres/pose_graph_3d_error_term.h"
#include "models/optimizer/ceres/types.h"

namespace avp_mapping {
class PGOBackEnd {
public:
  PGOBackEnd(YAML::Node &node);

  bool Optimize();
  void AddPose(const KeyFrame &keyframe);
  void AddConstraint(int index_begin, int index_end,
                     const Eigen::Matrix4d &T_be,
                     const std::vector<double> &info);
  void GetOptimizedPoses(std::deque<KeyFrame>& kfs);

private:
  bool SolveOptimizationProblem(ceres::Problem *problem);
  void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                                MapOfPoses *poses, ceres::Problem *problem);
  bool OutputPoses(std::string file_path, const MapOfPoses &poses);

private:
  bool save_pose_ = true;
  std::string save_dir_ = "./";

  MapOfPoses original_poses_;
  MapOfPoses poses_;
  VectorOfConstraints constraints_;
};
} // namespace avp_mapping

#endif // _PGO_BACK_END_H_