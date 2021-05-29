#include "cal_pose.h"

#include <opencv2/opencv.hpp>

#include "system_info.h"

inline int countNonZeroOfU8rows(const cv::Mat& m) {
  assert(m.type() == CV_8U);
  int sz = 0;
  for (int i = 0; i < m.rows; ++i) {
    if (m.at<uchar>(i, 0) != 0) ++sz;
  }
  return sz;
}

bool EpipolarPoser::calculate(cv::Mat& R, cv::Mat& t) {
  using namespace std;
  // compute Essential and H matrix
  cv::Mat e_inliers;
  auto E = cv::findEssentialMat(_kpts1, _kpts2, sys.camera_K(), cv::RANSAC,
                                sys.ransac_confidence_in_compute_H_E(),
                                sys.max_error_in_compute_F_E(), e_inliers);
  int n1 = countNonZeroOfU8rows(e_inliers);
  cv::Mat h_inliers;
  auto H = cv::findHomography(_kpts1, _kpts2, cv::RANSAC,
                              sys.max_error_in_compute_H(), h_inliers, 10000,
                              sys.ransac_confidence_in_compute_H_E());
  int n2 = countNonZeroOfU8rows(h_inliers);

  float H_E_ratio = float(n2) / float(n1);
  if (H_E_ratio < 0.75f && n1 >= sys.min_inliers_in_2d2d_matching()) {
    cv::recoverPose(E, _kpts1, _kpts2, sys.camera_K(), R, t, e_inliers);
    _inliers = e_inliers;
    return true;
  } else if (n2 >= sys.min_inliers_in_2d2d_matching()) {
    // TODO: recover pose from matrix H.
    cv::recoverPose(E, _kpts1, _kpts2, sys.camera_K(), R, t, e_inliers);
    _inliers = e_inliers;
    return true;
  }
  return false;
}

bool PnpPoser::calculate(cv::Mat& R, cv::Mat& t) {
  if (_map_pts.size() < 4) return false;
  cv::Mat rvec, tvec, inliers;
  // cv::solvePnP(_map_pts, _kpts, sys.camera_K(), sys.distort_parameter(),
  // rvec,
  //             tvec);
  cv::solvePnPRansac(
      _map_pts, _kpts, sys.camera_K(), sys.distort_parameter(), rvec, tvec,
      false, sys.ransac_times_in_pnp(), (float)sys.max_error_in_pnp(),
      sys.ransac_confidence_in_pnp(), inliers, cv::SOLVEPNP_EPNP);
  if (inliers.rows < sys.min_inlers_in_pnp()) return false;
  cv::Rodrigues(rvec, R);
  t = tvec;
  assert(inliers.type() == CV_32S);
  _inlier_idxes = inliers;
  return true;
}
