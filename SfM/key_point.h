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

struct KeyPoint_Impl {
  int64_t _mappoint_id = -1;  ///< corresponding mappoint's ID, default -1, -1
                              ///< means no related mappoint
  cv::KeyPoint _kp;           ///< cv::KeyPoint
};

/// \class KeyPoint
/// \brief a 2D key point of an image
/// \note
/// * I use impl-ptr for optimizing, when the key point is never used,
/// use clear() to release the heap-memory.\n
/// Use empty() to observe the impl-ptr whether empty.
/// *
class KeyPoint {
  sp<KeyPoint_Impl> _self;
  KeyPoint(bool use_impl);

 public:
  KeyPoint();
  bool empty() const;
  void clear();

  /// \brief transform the opencv keypoints to my-version keypoints
  /// \param _kps opencv keypoints
  /// \return
  static std::vector<KeyPoint> TransformFromCVKeypoints(
      const std::vector<cv::KeyPoint>& _kps);
  static KeyPoint FromCVkp(const cv::KeyPoint& kp);
  static cv::KeyPoint ToCVKp(const KeyPoint& kp);

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int ver);
  int64_t id() const;
  void id(int64_t id);
  Point2 pt() const;
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
