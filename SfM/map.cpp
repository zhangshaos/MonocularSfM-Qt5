#include "map.h"

#include <execution>
#include <fstream>

#include "db_init.h"

int64_t Map::addMapPoint(const MapPoint& pt) {
  ulock<smtx> lock(_mtx);
  int64_t id = pt.id();
  _mappts[id] = pt;
  return id;
}

bool Map::existedMapPoint(int64_t id) const {
  slock<smtx> lock(_mtx);
  return _mappts.count(id);
}

const MapPoint& Map::getMapPoint(int64_t id) const { return _mappts.at(id); }

MapPoint& Map::getMapPoint(int64_t id) { return _mappts.at(id); }

void Map::writeToBinPLY(const std::string& path) const {
  using namespace std;
  ofstream file(path, ios_base::out | ios_base::binary | ios_base::trunc);
  assert(file.is_open());
  file.imbue(locale("C"));

  file << "ply" << std::endl;
  file << "format binary_little_endian 1.0" << std::endl;
  file << "element vertex " << _mappts.size() << std::endl;
  // NOTE: if you set the std::local::global(local("zh_CN.utf-8")),
  // the number 1000 may be transformed to 1,000 for output !!!
  file << "property double x" << std::endl;
  file << "property double y" << std::endl;
  file << "property double z" << std::endl;
  // file << "property uchar red" << std::endl;
  // file << "property uchar green" << std::endl;
  // file << "property uchar blue" << std::endl;
  file << "end_header" << std::endl;

  slock<smtx> lock(_mtx);
  for (auto& p : _mappts) {
    int64_t id = p.first;
    auto& mp = p.second;
    cv::Vec3d xyz = mp.pos();
    file.write(reinterpret_cast<char*>(xyz.val), sizeof(xyz.val));
  }
  file.close();
}

void Map::writeToPLY(const std::string& path) const {
  using namespace std;
  ofstream file(path);
  assert(file.is_open());
  file.imbue(locale("C"));

  file << "ply" << std::endl;
  file << "format ascii 1.0" << std::endl;
  // NOTE: if you set the std::local::global(local("zh_CN.utf-8")),
  // the number 1000 may be transformed to 1,000 for output !!!
  file << "element vertex " << _mappts.size() << std::endl;
  file << "property double x" << std::endl;
  file << "property double y" << std::endl;
  file << "property double z" << std::endl;
  // file << "property uchar red" << std::endl;
  // file << "property uchar green" << std::endl;
  // file << "property uchar blue" << std::endl;
  file << "end_header" << std::endl;

  slock<smtx> lock(_mtx);
  for (auto& p : _mappts) {
    int64_t id = p.first;
    auto& mp = p.second;
    cv::Vec3d xyz = mp.pos();
    file << boost::format("%1% %2% %3%\n") % xyz[0] % xyz[1] % xyz[2];
  }
  file.close();
}

void Map::removeMapPoint(int64_t id) {
  ulock<smtx> lock(_mtx);
  _mappts.erase(id);
}

void Map::removeMapPoint(int64_t id, sp<DB>& db) {
  slock<smtx> lock(_mtx);
  using namespace std;
  // clear the relationship between bad point and key point
  getMapPoint(id).clearAllObservers(db);
  // delete the bad point, after delete, the obs is invalid!
  lock.unlock();
  removeMapPoint(id);
}

void Map::clear(sp<DB>& db) {
  slock<smtx> lock(_mtx);
  using namespace std;
  // clear all relationship between map points and their observers
  using Id_Mp = decltype(_mappts)::value_type;
  for_each(execution::par, _mappts.begin(), _mappts.end(),
           [&db](Id_Mp& id_mp) { id_mp.second.clearAllObservers(db); });
  // delete all map points
  lock.unlock();
  ulock<smtx> lock2(_mtx);
  _mappts.clear();
}

int64_t Map::size() const {
  slock<smtx> lock(_mtx);
  return (int64_t)_mappts.size();
}

std::vector<Point3> Map::getAllPoints() const {
  ulock<smtx> lock(_mtx);
  std::vector<Point3> point3s;
  for (auto& id_mp : _mappts) {
    point3s.emplace_back(id_mp.second.pos());
  }
  return point3s;
}
