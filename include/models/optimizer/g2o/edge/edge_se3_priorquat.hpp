/*
 * @Author: ding.yin
 * @Date: 2022-11-08 15:55:55
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-08 19:21:44
 */

#ifndef _EDGE_SE3_PRIORQUAT_H_
#define _EDGE_SE3_PRIORQUAT_H_

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

namespace g2o {
class EdgeSE3PriorQuat
    : public g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PriorQuat()
      : g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3>() {}

  void computeError() override {
    const g2o::VertexSE3 *v1 =
        static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    Eigen::Quaterniond estimate = Eigen::Quaterniond(v1->estimate().linear());
    if (estimate.w() < 0) {
      estimate.coeffs() = -estimate.coeffs();
    }
    _error = estimate.vec() - _measurement.vec();
  }

  void setMeasurement(const Eigen::Quaterniond &m) override {
    _measurement = m;
    if (m.w() < 0.0) {
      _measurement.coeffs() = -m.coeffs();
    }
  }

  virtual bool read(std::istream &is) override {
    Eigen::Quaterniond q;
    is >> q.w() >> q.x() >> q.y() >> q.z();
    setMeasurement(q);
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
    Eigen::Quaterniond q;
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = 0; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      }
    }
    return os.good();
  }
};

} // namespace g2o

#endif // _EDGE_SE3_PRIORQUAT_H_
