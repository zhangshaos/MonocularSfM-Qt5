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
  bool run(sp<Map> &map);

  sp<UsedImages> used_images() const { return _used_images; }

  int init_image() const { return _init_image; }
};

#endif  // !__monocularsfm_initialize_h__