/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:40:48
 */
#ifndef _CLOUD_DATA_H_
#define _CLOUD_DATA_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace avp_mapping {
class CloudData {
public:
  using POINT = pcl::PointXYZRGB;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

public:
  CloudData() : cloud_ptr(new CLOUD()) {}

public:
  double time = 0.0;
  CLOUD_PTR cloud_ptr;
};
} // namespace avp_mapping

#endif // _CLOUD_DATA_H_
