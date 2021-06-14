#include "key_point.h"

#include <algorithm>
#include <execution>
#include <opencv2/calib3d.hpp>

#include "global_config.h"

std::vector<KeyPoint> KeyPoint::TransformFromCVKeypoints(
    const std::vector<cv::KeyPoint>& _kps) {
  std::vector<KeyPoint> res(_kps.size());
  std::transform(std::execution::par, _kps.begin(), _kps.end(), res.begin(),
                 KeyPoint::FromCVkp);
  return res;
}

void UndistortPoint(Point2& pt2, const cv::Mat1d& K,
                    const std::vector<double>& distort) {
#ifdef REFINE_INTRINSIC
  using namespace std;
  using vec = vector<Point2>;
  vec dests;
  cv::undistortPoints(vec({pt2}), dests, K, distort, cv::noArray(), K);
  swap(dests[0], pt2);
#endif
}

void UndistortPoint(std::vector<Point2>& pt2s, const cv::Mat1d& K,
                    const std::vector<double>& distort) {
#ifdef REFINE_INTRINSIC
  using namespace std;
  using vec = vector<Point2>;
  vec dests;
  cv::undistortPoints(pt2s, dests, K, distort, cv::noArray(), K);
  swap(dests, pt2s);
#endif
}
