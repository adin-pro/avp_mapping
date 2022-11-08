/*
 * @Author: ding.yin
 * @Date: 2022-11-08 09:06:15
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-08 19:22:12
 */
#ifndef _EDGE_SE3_PRIORXYZ_H_
#define _EDGE_SE3_PRIORXYZ_H_

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

namespace g2o {
// 3: the dimension of edge is 3
// Eigen::Vector3d : the container of edge is vector3d
// g2o::VertexSE3: type of vertex can be connected to this edge
class EdgeSE3PriorXYZ
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PriorXYZ()
      : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {}
  void computeError() override {
    const g2o::VertexSE3 *v1 =
        static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    Eigen::Vector3d estimate = v1->estimate().translation();
    _error = estimate - _measurement; // xyz difference
  }

  void setMeasurement(const Eigen::Vector3d &m) override { _measurement = m; }

  virtual bool read(std::istream &is) override {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);
    setMeasurement(Eigen::Vector3d(v));
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(i, j) = information()(j, i);
      }
    }
    return true;
  }

  virtual bool write(std::ostream &os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      }
    }
    return os.good();
  }
};

} // namespace g2o

#endif // _EDGE_SE3_PRIORXYZ_H_