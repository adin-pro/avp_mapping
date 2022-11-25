/*
 * @Author: ding.yin
 * @Date: 2022-11-20 21:18:28
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-24 20:02:44
 */

#include "models/loop_close/semantic_list.hpp"
#include "glog/logging.h"

namespace avp_mapping {

// https://github.com/oleg-alexandrov/projects/blob/master/eigen/Kabsch.cpp

// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm

// The input 3D points are stored as columns.
Eigen::Affine3d SemanticList::Find3DAffineTransform(Eigen::Matrix3Xd in,
                                                    Eigen::Matrix3Xd out,
                                                    bool fixed_scale) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols() - 1; col++) {
    dist_in += (in.col(col + 1) - in.col(col)).norm();
    dist_out += (out.col(col + 1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out / dist_in;
  if (fixed_scale) {
    scale = 1.0;
  }
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col) -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale * (out_ctr - R * in_ctr);

  return A;
}

std::vector<double> SemanticList::coarse_semantic_coeff_ =
    std::vector<double>(AVPColors.size(), 1.0);
std::vector<double> SemanticList::find_semantic_coeff_ =
    std::vector<double>(AVPColors.size(), 1.0);

int SemanticList::half_width_ = 200;
int SemanticList::half_height_ = 200;
double SemanticList::scale_ = 0.05;

bool SemanticList::addSemanticNode(const std::vector<std::vector<double>> &vecs,
                                   AVPLabels type) {
  if (vecs.empty() || vecs[0].size() < 5)
    return false;
  // vec [x, y, width, height, area]
  int id = 0;
  for (auto &v : vecs) {
    SemanticNode sNode;
    sNode.type_ = type;
    sNode.px_ = v[0];
    sNode.py_ = v[1];
    sNode.width_ = v[2];
    sNode.height_ = v[3];
    sNode.area_ = v[4];
    sNode.id_ = id;
    nodes_list_.emplace_back(sNode);
    id++;
  }
  num_objects_[type] += static_cast<double>(vecs.size());
  return true;
}

double SemanticList::calcDistance(const SemanticNode &node1,
                                  const SemanticNode &node2) {
  return sqrt((node1.px_ - node2.px_) * (node1.px_ - node2.px_) +
              (node1.py_ - node2.py_) * (node1.py_ - node2.py_));
}

void SemanticList::procNodes() {
  for (size_t i = 0; i < nodes_list_.size(); ++i) {
    for (size_t j = i + 1; j < nodes_list_.size(); ++j) {
      auto &sNodeI = nodes_list_[i];
      auto &sNodeJ = nodes_list_[j];
      // euclidean distance
      double distance = SemanticList::calcDistance(sNodeI, sNodeJ);
      // update semantic node i
      sNodeI.dist_to_other_nodes_[sNodeJ.type_] += distance;
      sNodeI.num_other_nodes_[sNodeJ.type_] += 1.0;
      // udpate semantic node j
      sNodeJ.dist_to_other_nodes_[sNodeI.type_] += distance;
      sNodeJ.num_other_nodes_[sNodeI.type_] += 1.0;
    }
  }
}

void SemanticList::setId(int id) { id_ = id; }

void SemanticList::setTwc(const Eigen::Matrix4d &pose) { Twc_ = pose; }

void SemanticList::printInfo() {
  std::cout << "Frame " << id_ << std::endl;
  std::cout << "Twc " << std::endl;
  std::cout << Twc_ << std::endl;
  std::cout << "#number ";
  for (size_t i = 0; i < num_objects_.size(); ++i) {
    std::cout << num_objects_[i] << " ";
  }
  std::cout << std::endl;
}

double SemanticList::coarseSimilarity(const SemanticList &sListA,
                                      const SemanticList &sListB) {
  std::vector<double> vecA = sListA.num_objects_;
  std::vector<double> vecB = sListB.num_objects_;
  for (size_t i = 0; i < SemanticList::coarse_semantic_coeff_.size(); ++i) {
    vecA[i] *= coarse_semantic_coeff_[i];
    vecB[i] *= coarse_semantic_coeff_[i];
  }
  double distance = 0.0;
  for (size_t i = 0; i < vecA.size(); ++i) {
    distance += sqrt((vecA[i] - vecB[i]) * (vecA[i] - vecB[i]));
  }
  Eigen::Map<Eigen::VectorXd> vA(vecA.data(), vecA.size());
  Eigen::Map<Eigen::VectorXd> vB(vecB.data(), vecB.size());
  return vA.dot(vB) / (vA.norm() * vB.norm() + 1e-15) *
         (1.0 - distance / vA.norm());
}

double SemanticList::fineSimilarity(const SemanticList &sListA,
                                    const SemanticList &sListB) {
  double fine_similarity = 1.0;
  for (size_t iA = 0; iA < sListA.nodes_list_.size(); ++iA) {
    double max_sim = 0.0;
    if (sListA.nodes_list_[iA].type_ == AVPLabels::LANE_LINE ||
        sListA.nodes_list_[iA].type_ == AVPLabels::PARKING_LINE) {
      continue;
    }
    for (size_t iB = 0; iB < sListB.nodes_list_.size(); ++iB) {
      if (sListA.nodes_list_[iA].type_ == sListB.nodes_list_[iB].type_) {
        double node_sim = SemanticList::sNodeSimilarity(sListA.nodes_list_[iA],
                                                        sListB.nodes_list_[iB]);
        if (node_sim > max_sim) {
          max_sim = node_sim;
        }
      }
    }
    fine_similarity += max_sim;
  }
  return fine_similarity;
}

Eigen::Matrix4d SemanticList::getTransformation(SemanticList &sListA,
                                                SemanticList &sListB) {
  std::vector<std::pair<int, int>> point_pairs;
  for (size_t iA = 0; iA < sListA.nodes_list_.size(); ++iA) {
    double max_sim = 0.0;
    if (sListA.nodes_list_[iA].type_ == AVPLabels::LANE_LINE ||
        sListA.nodes_list_[iA].type_ == AVPLabels::PARKING_LINE) {
      continue;
    }
    for (size_t iB = 0; iB < sListB.nodes_list_.size(); ++iB) {
      if (sListA.nodes_list_[iA].type_ == sListB.nodes_list_[iB].type_) {
        double node_sim = SemanticList::sNodeSimilarity(sListA.nodes_list_[iA],
                                                        sListB.nodes_list_[iB]);
        if (node_sim > max_sim) {
          max_sim = node_sim;
          sListA.nodes_list_[iA].corr_id_ = iB;
        }
      }
    }
    point_pairs.push_back(
        std::pair<int, int>{iA, sListA.nodes_list_[iA].corr_id_});
    std::cout
        << sListA.nodes_list_[iA].getTypeStr() << " ("
        << sListA.nodes_list_[iA].px_ << ", " << sListA.nodes_list_[iA].py_
        << ")"
        << " ----> "
        << sListB.nodes_list_[sListA.nodes_list_[iA].corr_id_].getTypeStr()
        << " (" << sListB.nodes_list_[sListA.nodes_list_[iA].corr_id_].px_
        << ", " << sListB.nodes_list_[sListA.nodes_list_[iA].corr_id_].py_
        << ")" << std::endl;
  }

  Eigen::Matrix3Xd pointsA(3, point_pairs.size());
  Eigen::Matrix3Xd pointsB(3, point_pairs.size());

  for (size_t i = 0; i < point_pairs.size(); ++i) {
    SemanticNode &sA = sListA.nodes_list_[point_pairs[i].first];
    SemanticNode &sB = sListB.nodes_list_[point_pairs[i].second];
    pointsA.col(i) << (half_height_ - sA.py_) * scale_,
        (half_width_ - sA.px_) * scale_, 0;
    pointsB.col(i) << (half_height_ - sB.py_) * scale_,
        (half_width_ - sB.px_) * scale_, 0;
  }
  LOG(INFO) << "Affine3d Matrix";
  Eigen::Affine3d TAB =
      SemanticList::Find3DAffineTransform(pointsA, pointsB, true);
  std::cout << TAB.matrix() << TAB.linear().eulerAngles(0, 1, 2) << std::endl;
  return TAB.matrix();
}

double SemanticList::sNodeSimilarity(const SemanticNode &sNodeA,
                                     const SemanticNode &sNodeB) {
  double euler_distance_square_sum = 0.0;
  for (size_t i = 0; i < sNodeA.dist_to_other_nodes_.size(); ++i) {
    euler_distance_square_sum +=
        SemanticList::find_semantic_coeff_[i] *
        sqrt((sNodeA.dist_to_other_nodes_[i] - sNodeB.dist_to_other_nodes_[i]) *
             (sNodeA.dist_to_other_nodes_[i] - sNodeB.dist_to_other_nodes_[i]) /
             (sNodeA.num_other_nodes_[i] + 1e-15));
  }
  return log(1.0 + 100.0 / (euler_distance_square_sum + 1e-15));
}

} // namespace avp_mapping