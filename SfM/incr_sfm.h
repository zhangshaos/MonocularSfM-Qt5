//
/// \file src/control/incr_sfm.h
///
/// \author zxm
/// \email xingmingzhangssr\gmail
/// \version 0.1
//

#ifndef __monocular_incr_sfm__
#define __monocular_incr_sfm__

#include "common_type.h"
#include "initialize.h"

/// \brief increasing SfM(structure from motion)
/// \note
/// * you must to use \class Initialize to initialize the global
///   \class DB and \class Map before using this class.
struct IncrSfM {
  sp<DB> db;
  sp<Map> map;
  sp<UsedImages> used_images;
  int init_image;

  IncrSfM(const sp<DB> &_db, const sp<Map> &_map, const sp<UsedImages> &_imgs,
          int init_image_id)
      : db(_db), map(_map), used_images(_imgs), init_image(init_image_id) {}

  /// \brief run a pass of IncrSfM
  /// \return TRUE if okay, otherwise FALSE
  /// \retval bool
  [[deprecated("use runPnp[MapPoints, BA] instead")]] bool run();

  /**
   * @brief calculate current image's pose(Tcw) using Pnp algorithm
   * @param[in] id the id of image to calculate its pose
   * @return TRUE if succeeding, else FALSE
   */
  bool runPnp(int id);

  [[maybe_unused]] cv::Matx33f dbg_Rcw;
  [[maybe_unused]] cv::Matx31f dbg_tcw;
  [[maybe_unused]] void debugPose(const cv::Mat1d &R, const cv::Mat1d &t);
  [[maybe_unused]] std::pair<cv::Mat1f, cv::Mat1f> debugPose() const;

  [[maybe_unused]] std::vector<Point3> dbg_pts;
  [[maybe_unused]] void debugPoints(const std::vector<Point3> &pt3s);
  [[maybe_unused]] std::vector<Point3> debugPoints() const;

  /**
   * @brief create new map points
   * @param[in] id the id of current image
   * @return True if succeeding, else False
   */
  bool runMapPoints(int id);

  /**
   * @brief run Bundle Adjustment for optimizing map points and images' pose
   * @param[in] local
   * @param[in] none_pose if true, which means not optimize pose
   * @param[in] cur_image if -1, which means all poses will be optimized\n
   *            if image is current pose(non -1), which means all poses exclude
   *            initial and current pose
   */
  void runBA(bool local = true, bool none_pose = true, int cur_image = -1);

  void saveMap(const std::string &path) const;
  void savePoses(const std::string &path) const;

 private:
  /// \brief track map points and key points saw by the image whose id is \p img
  /// \param[out] map_pts
  /// \param[out] kpts
  /// \note
  void trackObservers(int img, std::vector<Point3> &map_pts,
                      std::vector<Point2> &kpts) const;
};

#endif  // !__monocular_incr_sfm__
