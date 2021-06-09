//
/// \file src/view/map.h
///
/// \author zxm
/// \email xingmingzhangssr\gmail
/// \version 0.1
//

#ifndef __monocular_map_h__
#define __monocular_map_h__

#include <unordered_map>

#include "map_point.h"

class Map {
  std::unordered_map<int64_t, MapPoint> _mappts;
  mutable smtx _mtx;

 public:
  /// \brief add a new \class MapPoint to this map
  /// \param[in] pt new MapPoint
  /// \return id of the new MapPoint
  /// \retval int64_t
  int64_t addMapPoint(const MapPoint& pt);

  /// \brief
  /// \param id
  /// \return
  bool existedMapPoint(int64_t id) const;

  /// \brief
  /// \param id
  /// \return
  const MapPoint& getMapPoint(int64_t id) const;
  MapPoint& getMapPoint(int64_t id);

  /// \brief write all map points to binary .PLY file
  /// \param path specify the filesystem path of .PLY file
  void writeToBinPLY(const std::string& path) const;
  void writeToPLY(const std::string& path) const;

  /// \brief remove a map point whose id is \p id
  /// \param id map point's id
  void removeMapPoint(int64_t id);

  /// \brief remove map point \id and its all observers in \p db
  /// \param[in] id
  /// \param[in, out] db
  void removeMapPoint(int64_t id, sp<DB>& db);

  /// \brief clear all map points and their observers
  /// \param[in, out] db
  void clear(sp<DB>& db);

  int64_t size() const;

  /**
   * @brief export map's all points' position
   * @return vector<Point3>
   */
  std::vector<Point3> getAllPoints() const;
};

#endif  // !__monocular_map_h__
