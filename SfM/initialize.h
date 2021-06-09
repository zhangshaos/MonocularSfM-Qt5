//
/// \file src/control/initialize.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocularsfm_initialize_h__
#define __monocularsfm_initialize_h__

#include <memory>
#include <unordered_set>

#include "common_type.h"
#include "db_init.h"
#include "map.h"

/// \class Initialize
/// \brief initialzing the map
/// \note
/// *
class Initialize {
  sp<DB> _db;
  sp<UsedImages> _used_images{new UsedImages};
  int _init_image = -1;

 public:
  Initialize(const sp<DB> &db) : _db(db) {}

  /// \brief run initializing
  /// \param[out] map initialed global map
  /// \return TRUE means initialing is ok, FALSE means failed
  /// \note
  /// * the \p map will be reset whether it owm \class Map or not
  [[deprecated("use runInStep instead")]]
  bool run(sp<Map> &map);

  /**
   * @brief initialize in one step
   * @param image_1
   * @param image_2
   * @param map
   * @return
   */
  bool runInStep(int image_1, int image_2, sp<Map> &map);

  sp<UsedImages> used_images() const { return _used_images; }

  int init_image() const { return _init_image; }
};

/// \brief src/control/initialize.h
/// \ref GetMatchedKeypointsPairFromDB(...) parameter
struct InitialMatchedParam {
  const DB &db;
  int pr1, pr2;
  std::vector<int> ptIdxes1, ptIdxes2;
  /// \brief constructor
  /// \param _db
  /// \param img1
  /// \param img2
  InitialMatchedParam(const DB &_db, int img1, int img2)
      : db(_db), pr1(img1), pr2(img2) {}
};

/**
 * @brief get matched Point2D from global DB
 * @param pm \see \class InitialMatchedParam
 * @return
 */
std::pair<std::vector<Point2>, std::vector<Point2>>
GetMatchedKeypointsPairFromDB(InitialMatchedParam &pm);

#endif  // !__monocularsfm_initialize_h__