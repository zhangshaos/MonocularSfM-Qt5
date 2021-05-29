#include "key_point.h"

#include <algorithm>
#include <execution>

#include "global_config.h"

std::vector<KeyPoint> KeyPoint::TransformFromCVKeypoints(
    const std::vector<cv::KeyPoint>& _kps) {
  std::vector<KeyPoint> res(_kps.size());
  std::transform(std::execution::par, _kps.begin(), _kps.end(),
                 res.begin(), KeyPoint::FromCVkp);
  return res;
}
