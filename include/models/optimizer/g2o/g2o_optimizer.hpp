/*
 * @Author: ding.yin
 * @Date: 2022-11-08 09:04:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-08 20:22:19
 */
#ifndef _G2O_OPTIMIZER_H_
#define _G2O_OPTIMIZER_H_

#include "models/optimizer/g2o/edge/edge_se3_priorquat.hpp"
#include "models/optimizer/g2o/edge/edge_se3_priorxyz.hpp"
#include "models/optimizer/optimizer_interface.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"

namespace avp_mapping {

class G2OGraphOptimizer : public OptimizerInterface {
public:
  G2OGraphOptimizer(const std::string &solver_type = "lm_var");

  bool optimize() override;

  bool getOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) override;

  int getNodeNum() override;

  void setEdgeRobustKernel(std::string robust_kernel_name,
                           double robust_kernel_size) override;

  void addSE3Node(const Eigen::Isometry3f &pose, bool need_fix) override;

  void addSE3Edge(int vertex_index1, int vertex_index2,
                  const Eigen::Isometry3f &relative_pose,
                  const Eigen::VectorXf noise) override;

  void addSE3PriorXYZEdge(int se3_vertex_index, const Eigen::Vector3f &xyz,
                          const Eigen::VectorXf noise) override;

  void addSE3PriorQuaternionEdge(int se3_vertex_index,
                                 const Eigen::Quaternionf &quat,
                                 Eigen::VectorXf noise) override;

  void setMaximumIteration(int max_iter) override;

private:
  Eigen::MatrixXf calcSE3EdgeInfoMat(Eigen::VectorXf noise);
  Eigen::MatrixXf calcSE3PriorQuatEdgeInfoMat(Eigen::VectorXf noise);
  Eigen::MatrixXf calcDiagMatrix(Eigen::VectorXf noise);
  void addRobustKernel(g2o::OptimizableGraph::Edge *edge,
                       const std::string &kernel_type, double kernel_size);

private:
  g2o::RobustKernelFactory *robust_kernel_factory_;
  std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;
  std::string robust_kernel_name_;
  double robust_kernel_size_;
  bool need_robust_kernel_ = false;
};

} // namespace avp_mapping

#endif // _G2O_OPTIMIZER_H_