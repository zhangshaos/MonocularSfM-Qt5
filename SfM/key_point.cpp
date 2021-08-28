#include "key_point.h"

#include <algorithm>
#include <execution>
#include <opencv2/calib3d.hpp>

#include "global_config.h"

KeyPoint::KeyPoint(bool use_impl) : _self(new KeyPoint_Impl) {}

KeyPoint::KeyPoint() {}

bool KeyPoint::empty() const { return _self.get() == nullptr; }

void KeyPoint::clear() { _self.reset(); }

std::vector<KeyPoint> KeyPoint::TransformFromCVKeypoints(
    const std::vector<cv::KeyPoint>& _kps) {
  std::vector<KeyPoint> res(_kps.size(), KeyPoint(false));
  std::transform(std::execution::par, _kps.begin(), _kps.end(), res.begin(),
                 KeyPoint::FromCVkp);
  return res;
}

KeyPoint KeyPoint::FromCVkp(const cv::KeyPoint& kp) {
  KeyPoint p(true);
  p._self->_kp = kp;
  return p;
}

cv::KeyPoint KeyPoint::ToCVKp(const KeyPoint& kp) { return kp._self->_kp; }

int64_t KeyPoint::id() const { return _self->_mappoint_id; }

void KeyPoint::id(int64_t id) { _self->_mappoint_id = id; }

Point2 KeyPoint::pt() const { return _self->_kp.pt; }

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
