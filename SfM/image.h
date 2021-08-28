//
/// \file src/model/image.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocularsfm_image_h__
#define __monocularsfm_image_h__

#include <atomic>
#include <filesystem>
#include <opencv2/core.hpp>

#include "key_point.h"

struct Image_impl {
  int id = -1;              ///< ID of an image, default -1
  uint64_t R_ba_times = 0;  ///< optimized Rcw times of bundle adjustment
  uint64_t t_ba_times = 0;  ///< optimized tcw times of bundle adjustment
  std::string path;         ///< path of the image file
  cv::Mat gray_mat,
      rgb_mat;                 ///< color or gray mat returned by cv::imread()
  std::vector<KeyPoint> kpts;  ///< all keypoints of this image
  cv::Mat descp;               ///< keypoints' descriptor
  cv::Mat Rcw, tcw;            ///< pose(Tcw = Rcw3x3 | tcw3x1) of this image
};

/// \class Image
/// \brief image of database
/// \note
/// *
class Image {
  sp<Image_impl> self;

 public:
  Image();

  template <typename Archive>
  void serialize(Archive &ar, const unsigned int ver);

  /// \brief get a new Image ID to set image._id
  /// \param none
  /// \return new id
  /// \note
  /// * This funtion can be used in multi-thread
  static void GetNewImageID(int &id);

  void id(int id);
  int id() const;
  const cv::Mat &gray_mat() const;
  void gray_mat(const cv::Mat &m);
  const cv::Mat &rgb_mat() const;
  void rgb_mat(const cv::Mat &m);
  const std::vector<KeyPoint> &kpts() const;
  std::vector<KeyPoint> &kpts();
  void kpts(std::vector<KeyPoint> &&kps);
  const cv::Mat &descp() const;
  cv::Mat &descp();
  void descp(const cv::Mat &m);
  const cv::Mat &Rcw() const;
  const cv::Mat &tcw() const;
  void Rcw(const cv::Mat &Rcw);
  void tcw(const cv::Mat &tcw);
  const std::string &path() const;
  void path(const std::string &path);
  uint64_t R_ba_times() const;
  uint64_t t_ba_times() const;
  void inc_R_ba_times();
  void inc_t_ba_times();

  /// \brief get matched map point's id of \p kp_idx-th key point
  /// \param kp_idx
  /// \return matched map point's id
  int64_t getMapptOfKpt(int kp_idx) const;

  /// \brief set matched \p mp_idx-th map point's id of \p kp_idx-th key point
  /// \param kp_idx
  /// \param mp_idx
  void setMapptOfKpt(int kp_idx, int64_t mp_idx);
};

#endif  // !__monocularsfm_image_h__