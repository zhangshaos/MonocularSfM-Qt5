//
/// \file src/control/cal_pose.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __mococular_cal_pose_h__
#define __mococular_cal_pose_h__

#include <vector>

#include "common_type.h"
#include "image.h"

/// \brief the interface of calculating rot matrix and translation
struct I_CalculatePose {
  /// \brief
  /// \param[out] R rot matrix of 3x3
  /// \param[out] t translation vector of 3x1
  /// \return true means ok, false means calculating pose failed
  virtual bool calculate(cv::Mat &R, cv::Mat &t) = 0;
  virtual ~I_CalculatePose() {}
};

class EpipolarPoser : public I_CalculatePose {
  const std::vector<Point2> &_kpts1, &_kpts2;
  cv::Mat &_inliers;

 public:
  /// \brief constructor
  /// \param kpts1 the first key points vector reference
  /// \param kpts2 the second key points vector reference
  /// \param inliers the inlier when compute pose reference
  /// \note
  /// * this class only own the reference to \p kpts1 and \p kpts2,
  ///   which means you must be concerned about their lifetime
  EpipolarPoser(const std::vector<Point2> &kpts1,
                const std::vector<Point2> &kpts2, cv::Mat &inliers)
      : _kpts1(kpts1), _kpts2(kpts2), _inliers(inliers) {}

  /// \brief
  /// \param[out] R rot matrix of 3x3
  /// \param[out] t translation vector of 3x1
  /// \return true means ok, false means calculating pose failed
  bool calculate(cv::Mat &R, cv::Mat &t) override;
};

class PnpPoser : public I_CalculatePose {
  const std::vector<Point3> &_map_pts;
  const std::vector<Point2> &_kpts;
  cv::Mat1i &_inlier_idxes;
  bool _existed;

 public:
  /// \brief constructor
  /// \param map_pts the map points vector reference
  /// \param kpts the key points vector reference
  /// \param inliers
  /// \param existed if true, use R|t with default value
  /// \note
  /// * this class only own the reference to \p map_pts and \p kpts,
  ///   which means you must be concerned about their lifetime
  PnpPoser(const std::vector<Point3> &map_pts, const std::vector<Point2> &kpts,
           cv::Mat1i &inliers, bool existed = false)
      : _map_pts(map_pts),
        _kpts(kpts),
        _inlier_idxes(inliers),
        _existed(existed) {}

  /// \brief
  /// \param[out] R rot matrix of 3x3
  /// \param[out] t translation vector of 3x1
  /// \return true means ok, false means calculating pose failed
  bool calculate(cv::Mat &R, cv::Mat &t) override;
};

#endif  // !__mococular_cal_pose_h__
