/*
 * @Author: ding.yin
 * @Date: 2022-11-25 14:34:48
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 16:39:43
 */

#include "mapping/back_end/pgo_back_end.hpp"

#include <fstream>

namespace avp_mapping {

//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)

bool PGOBackEnd::SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void PGOBackEnd::BuildOptimizationProblem(
    const VectorOfConstraints &constraints, MapOfPoses *poses,
    ceres::Problem *problem) {
  CHECK(poses != NULL);
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = NULL;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  for (VectorOfConstraints::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const Constraint3d &constraint = *constraints_iter;

    MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

    problem->AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
    problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  MapOfPoses::iterator pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool PGOBackEnd::OutputPoses(std::string filename, const MapOfPoses &poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::
           const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose3d, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const int, Pose3d>>>::
        value_type &pair = *poses_iter;
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}

bool PGOBackEnd::Optimize() {

  LOG(INFO) << "Number of poses: " << poses_.size() << '\n';
  LOG(INFO) << "Number of constraints: " << constraints_.size() << '\n';

  ceres::Problem problem;
  BuildOptimizationProblem(constraints_, &poses_, &problem);

  if (!SolveOptimizationProblem(&problem)) {
    LOG(ERROR) << "The solve was not successful, exiting.";
    return false;
  }
  if (save_pose_) {
    CHECK(OutputPoses(save_dir_ + "poses_original.txt", original_poses_))
        << "Error outputting to poses_original.txt";
    CHECK(OutputPoses(save_dir_ + "poses_optimized.txt", poses_))
        << "Error outputting to poses_original.txt";
  }
  return true;
}

PGOBackEnd::PGOBackEnd(YAML::Node &node) {
  save_pose_ = node["save_pose"].as<bool>();
  save_dir_ = node["save_dir"].as<std::string>();
  poses_.clear();
  original_poses_.clear();
  constraints_.clear();
}

void PGOBackEnd::AddPose(const KeyFrame &keyframe) {
  if (poses_.find(keyframe.index) != poses_.end()) {
    LOG(ERROR) << "Duplicate Vertex With ID " << keyframe.index;
  }
  Pose3d pose3d;
  pose3d.p.x() = keyframe.pose(0, 3);
  pose3d.p.y() = keyframe.pose(1, 3);
  pose3d.p.z() = keyframe.pose(2, 3);
  Eigen::Quaterniond q;
  q = keyframe.pose.block<3, 3>(0, 0);
  pose3d.q.x() = q.x();
  pose3d.q.y() = q.y();
  pose3d.q.z() = q.z();
  pose3d.q.w() = q.w();
  pose3d.time = keyframe.time;

  poses_[keyframe.index] = pose3d;
  Pose3d pose_origin = pose3d;
  original_poses_[keyframe.index] = pose_origin;
}

void PGOBackEnd::AddConstraint(int index_begin, int index_end,
                               const Eigen::Matrix4d &T_be,
                               const std::vector<double> &info) {
  Constraint3d constraint;
  constraint.id_begin = index_begin;
  constraint.id_end = index_end;
  Pose3d pose3d;
  pose3d.p.x() = T_be(0, 3);
  pose3d.p.y() = T_be(1, 3);
  pose3d.p.z() = T_be(2, 3);
  Eigen::Quaterniond q;
  q = T_be.block<3, 3>(0, 0);
  pose3d.q.x() = q.x();
  pose3d.q.y() = q.y();
  pose3d.q.z() = q.z();
  pose3d.q.w() = q.w();
  constraint.t_be = pose3d;
  assert(info.size() == 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        constraint.information(i, j) = info[i];
      } else {
        constraint.information(j, i) = 0.0;
      }
    }
  }
  constraints_.emplace_back(constraint);
}

void PGOBackEnd::GetOptimizedPoses(std::deque<KeyFrame> &kfs) {
  kfs.clear();
  for (auto &pa : poses_) {
    KeyFrame keyframe;
    keyframe.time = pa.second.time;
    keyframe.index = pa.first;
    keyframe.pose(0, 3) = pa.second.p.x();
    keyframe.pose(1, 3) = pa.second.p.y();
    keyframe.pose(2, 3) = pa.second.p.z();
    keyframe.pose.block<3, 3>(0, 0) = pa.second.q.toRotationMatrix();
    kfs.emplace_back(keyframe);
  }
}

} // namespace avp_mapping