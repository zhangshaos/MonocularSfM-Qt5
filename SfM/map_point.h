//
/// \file src/view/map_point.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocular_map_point_h__
#define __monocular_map_point_h__

#include "common_type.h"

/// \brief extern class ref
/// \see \class DB in db_init.h
class DB;

/// \brief \class MapPoint use this class to identify where that
/// map point can be see by \class Image
/// \note
/// *
/// *
class Observer {
  int _image_id = -1;
  int _kp_idx = -1;

 public:
  Observer(int image_id, int kp_idx) : _image_id(image_id), _kp_idx(kp_idx) {}

  int image_id() const { return _image_id; }
  int kp_idx() const { return _kp_idx; }
};

class MapPoint {
  struct MapPoint_impl {
    int64_t id = -1;
    Point3 pos;
    std::vector<Observer> obs;
  };

  static void GetNewMapPointID(int64_t& id);

  sp<MapPoint_impl> self{new MapPoint_impl};

 public:
  MapPoint() = default;
  MapPoint(const Point3& pos) {
    GetNewMapPointID(self->id);
    self->pos = pos;
  }

  int64_t id() const { return self->id; }
  const Point3& pos() const { return self->pos; }
  Point3& pos() { return self->pos; }
  const std::vector<Observer>& obs() const { return self->obs; }

  /// \brief 
  /// \param pos
  /// \param obs array of <image_id, key_point_idx>
  /// \param db 
  /// \return
  static MapPoint create(const Point3& pos,
                         const std::vector<std::pair<int, int>>& obs,
                         sp<DB>& db);

  /// \brief add a observer seeing this map point
  /// \param image_id the image saw this map point
  /// \param kp_idx the index of keypoints of the \p image
  void addObserver(int image_id, int kp_idx) {
    self->obs.emplace_back(image_id, kp_idx);
  }

  /// \brief clear all observers in \p db
  /// \param db 
  void clearAllObservers(sp<DB>& db);
};

#endif  // !__monocular_map_h__
