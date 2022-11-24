/*
 * @Author: ding.yin
 * @Date: 2022-11-07 14:56:08
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-08 20:25:39
 */

#ifndef _OPTIMIZER_INTERFACE_H_
#define _OPTIMIZER_INTERFACE_H_

#include <deque>
#include <string>

#include <Eigen/Dense>

namespace avp_mapping {

class OptimizerInterface {

public:
  OptimizerInterface() = default;
  virtual ~OptimizerInterface(){};

  virtual bool optimize() = 0;

  virtual bool
  getOptimizedPose(std::deque<Eigen::Matrix4d> &optimized_poses) = 0;

  virtual int getNodeNum() = 0;

  virtual void setEdgeRobustKernel(std::string robust_kernel_name,
                                   double robust_kernel_size) = 0;

  virtual void addSE3Node(const Eigen::Isometry3d &pose, bool need_fix) = 0;

  virtual void addSE3Edge(int vertex_index1, int vertex_index2,
                          const Eigen::Isometry3d &relative_pose,
                          const Eigen::VectorXd noise) = 0;

  virtual void addSE3PriorXYZEdge(int se3_vertex_index,
                                  const Eigen::Vector3d &xyz,
                                  const Eigen::VectorXd noise) = 0;

  virtual void addSE3PriorQuaternionEdge(int se3_vertex_index,
                                         const Eigen::Quaterniond &quat,
                                         Eigen::VectorXd noise) = 0;

  virtual void setMaximumIteration(int max_iter) = 0;

  int max_iteration_num_ = 30;

};

} // namespace avp_mapping

#endif // _OPTIMIZER_INTERFACE_H_