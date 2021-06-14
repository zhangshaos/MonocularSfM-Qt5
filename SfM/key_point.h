//
/// \file src/model/key_point.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocularsfm_keypoint_h__
#define __monocularsfm_keypoint_h__

#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

#include "common_type.h"

/// \class KeyPoint
/// \brief a 2D key point of an image
/// \note
/// *
/// *
class KeyPoint {
  int64_t _mappoint_id = -1;  ///< corresponding mappoint's ID, default -1, -1
                              ///< means no related mappoint
  cv::KeyPoint _kp;           ///< cv::KeyPoint

 public:
  /// \brief transform the opencv keypoints to my-version keypoints
  /// \param[in] _kps opencv keypoints
  /// \return my-version keypoints
  static std::vector<KeyPoint> TransformFromCVKeypoints(
      const std::vector<cv::KeyPoint>& _kps);
  static KeyPoint FromCVkp(const cv::KeyPoint& kp) {
    KeyPoint p;
    p._kp = kp;
    return p;
  }
  static cv::KeyPoint ToCVKp(const KeyPoint& kp) { return kp._kp; }

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int ver);
  int64_t id() const { return _mappoint_id; }
  void id(int64_t id) { _mappoint_id = id; }
  Point2 pt() const { return Point2(_kp.pt.x, _kp.pt.y); }
};

/**
 * @brief undistort \p pt2 to ideal point(u, v)
 * @param pt2 distorted point(u, v)
 * @param K
 * @param distort k1, k2, p1, p2
 * @return
 */
void UndistortPoint(Point2& pt2, const cv::Mat1d& K,
                    const std::vector<double>& distort);

/**
 * @brief undistort \p pt2s to ideal points
 * @param pt2s distorted points
 * @param K
 * @param distort k1, k2, p1, p2
 * @return
 */
void UndistortPoint(std::vector<Point2>& pt2s, const cv::Mat1d& K,
                    const std::vector<double>& distort);

#endif
