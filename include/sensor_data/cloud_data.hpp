/*
 * @Author: ding.yin
 * @Date: 2022-10-02 20:34:31
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:20:01
 */
/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
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
