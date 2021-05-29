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
  bool run();

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
