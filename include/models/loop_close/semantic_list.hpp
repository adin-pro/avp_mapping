/*
 * @Author: ding.yin
 * @Date: 2022-11-20 20:20:46
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-24 20:01:20
 */

#ifndef _SEMANTICLIST_H_
#define _SEMANTICLIST_H_

#include <string>
#include <vector>

#include "models/loop_close/semantic_node.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace avp_mapping {

struct SemanticList {
public:
  SemanticList() = default;
  void printInfo();
  bool addSemanticNode(const std::vector<std::vector<double>> &vecs,
                       AVPLabels type);
  void procNodes();
  void setId(int id);
  void setTwc(const Eigen::Matrix4d &pose);
  static double calcDistance(const SemanticNode &node1,
                             const SemanticNode &node2);
  static double coarseSimilarity(const SemanticList &sListA,
                                 const SemanticList &sListB);
  static double fineSimilarity(const SemanticList &sListA,
                               const SemanticList &sListB);
  static Eigen::Matrix4d getTransformation(SemanticList &sListA,
                                           SemanticList &sListB);

  static double sNodeSimilarity(const SemanticNode &sNodeA,
                                const SemanticNode &sNodeB);

  static Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in,
                                               Eigen::Matrix3Xd out,
                                               bool fixed_scale = true);

public:
  int id_ = -1;
  static int half_width_;
  static int half_height_;
  static double scale_;
  Eigen::Matrix4d Twc_ = Eigen::Matrix4d::Identity();
  std::vector<double> num_objects_ = std::vector<double>(AVPColors.size(), 0.0);
  static std::vector<double> coarse_semantic_coeff_;
  static std::vector<double> find_semantic_coeff_;
  std::vector<SemanticNode> nodes_list_;
};

} // namespace avp_mapping

#endif // _SEMANTICLIST_H_
