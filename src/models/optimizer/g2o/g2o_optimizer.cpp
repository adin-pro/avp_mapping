/*
 * @Author: ding.yin
 * @Date: 2022-11-08 17:16:23
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 20:53:15
 */

#include "models/optimizer/g2o/g2o_optimizer.hpp"

#include "g2o/core/optimization_algorithm_factory.h"
#include "tools/tic_toc.hpp"
#include "glog/logging.h"

namespace avp_mapping {

G2OGraphOptimizer::G2OGraphOptimizer(const std::string &solver_type) {
  graph_ptr_.reset(new g2o::SparseOptimizer());
  g2o::OptimizationAlgorithmFactory *solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm *solver =
      solver_factory->construct(solver_type, solver_property);
  graph_ptr_->setAlgorithm(solver);
  if (!graph_ptr_->solver()) {
    LOG(ERROR) << "G2O Optimizer Initialization Failed!";
  }
  robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

bool G2OGraphOptimizer::optimize() {
  static int optimize_cnt = 0;
  if (graph_ptr_->edges().size() < 1) {
    return false;
  }
  TicToc optimize_time;
  graph_ptr_->initializeOptimization();
  graph_ptr_->computeInitialGuess();
  graph_ptr_->computeActiveErrors();
  graph_ptr_->setVerbose(false);

  double chi2 = graph_ptr_->chi2();
  int iterations = graph_ptr_->optimize(max_iteration_num_);
  LOG(INFO) << std::endl
            << "------ Iteration " << optimize_cnt << " finished ------"
            << std::endl
            << "Number of vertices: " << graph_ptr_->vertices().size()
            << " Number of edges: " << graph_ptr_->edges().size() << std::endl
            << "Number of Iterations: " << iterations << " / "
            << max_iteration_num_ << std::endl
            << "Time cost: " << optimize_time.toc() << std::endl
            << "Error Difference: " << chi2 << " --> " << graph_ptr_->chi2()
            << std::endl;
  return true;
}

bool G2OGraphOptimizer::getOptimizedPose(
    std::deque<Eigen::Matrix4d> &optimized_pose) {
  optimized_pose.clear();
  int vertex_num = graph_ptr_->vertices().size();
  for (int i = 0; i < vertex_num; ++i) {
    g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(i));
    Eigen::Isometry3d pose = v->estimate();
    optimized_pose.push_back(pose.matrix());
  }
  return true;
}

int G2OGraphOptimizer::getNodeNum() { return graph_ptr_->vertices().size(); }

void G2OGraphOptimizer::addSE3Node(const Eigen::Isometry3d &pose,
                                   bool need_fix) {
  g2o::VertexSE3 *vertex(new g2o::VertexSE3());
  vertex->setId(graph_ptr_->vertices().size());
  vertex->setEstimate(pose);
  if (need_fix) {
    vertex->setFixed(true);
  }
  graph_ptr_->addVertex(vertex);
}

void G2OGraphOptimizer::setEdgeRobustKernel(std::string robust_kernel_name,
                                            double robust_kernel_size) {
  robust_kernel_name_ = robust_kernel_name;
  robust_kernel_size_ = robust_kernel_size;
  need_robust_kernel_ = true;
}

void G2OGraphOptimizer::addSE3Edge(int vertex_index1, int vertex_index2,
                                   const Eigen::Isometry3d &relative_pose,
                                   const Eigen::VectorXd noise) {
  Eigen::MatrixXd information_matrix = calcSE3EdgeInfoMat(noise);
  g2o::VertexSE3 *v1 =
      dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(vertex_index1));
  g2o::VertexSE3 *v2 =
      dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(vertex_index2));
  g2o::EdgeSE3 *edge(new g2o::EdgeSE3);
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph_ptr_->addEdge(edge);
  if(need_robust_kernel_) {
    addRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
  }
}

Eigen::MatrixXd G2OGraphOptimizer::calcSE3EdgeInfoMat(Eigen::VectorXd noise) {
  Eigen::MatrixXd information_mat = Eigen::MatrixXd::Identity(6, 6);
  information_mat = calcDiagMatrix(noise);
  return information_mat;
}

void G2OGraphOptimizer::addRobustKernel(g2o::OptimizableGraph::Edge * edge, const std::string& kernel_type, double kernel_size) {
  if (kernel_type == "NONE") {
    return;
  }
  g2o::RobustKernel * kernel = robust_kernel_factory_->construct(kernel_type);
  if (kernel == nullptr) {
    LOG(ERROR) << "Invalid Robust Kernel Type --> " << kernel_type;
    return;
  }
  kernel->setDelta(kernel_size);
  edge->setRobustKernel(kernel);
}

Eigen::MatrixXd G2OGraphOptimizer::calcDiagMatrix(Eigen::VectorXd noise) {
  Eigen::MatrixXd infomration_mat = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
  for (int i = 0; i < noise.rows(); ++i) {
    infomration_mat(i, i) /= noise(i);
  }
  return infomration_mat;
}

void G2OGraphOptimizer::addSE3PriorXYZEdge(int se3_vertex_index, const Eigen::Vector3d &xyz, Eigen::VectorXd noise) {
  Eigen::MatrixXd info_mat = calcDiagMatrix(noise);
  g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*> (graph_ptr_->vertex(se3_vertex_index));
  g2o::EdgeSE3PriorXYZ * edge(new g2o::EdgeSE3PriorXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(info_mat);
  edge->vertices()[0] = v_se3;
  graph_ptr_->addEdge(edge);
}

void G2OGraphOptimizer::addSE3PriorQuaternionEdge(int se3_vetex_index, const Eigen::Quaterniond & quat, Eigen::VectorXd noise) {
  Eigen::MatrixXd info_mat = calcDiagMatrix(noise);
  g2o::VertexSE3 * v_se3 = dynamic_cast<g2o::VertexSE3*> (graph_ptr_->vertex(se3_vetex_index));
  g2o::EdgeSE3PriorQuat * edge(new g2o::EdgeSE3PriorQuat());
  edge->setMeasurement(quat);
  edge->setInformation(info_mat);
  edge->vertices()[0] = v_se3;
  graph_ptr_->addEdge(edge);
}

void G2OGraphOptimizer::setMaximumIteration(int max_iter) {
  max_iteration_num_ = max_iter;
}

} // namespace avp_mapping