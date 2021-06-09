#include "map_point.h"

#include <atomic>
#include <execution>

#include "db_init.h"

struct MapPoint_impl {
  // the optimized times of bundle adjustment
  uint64_t ba_times = 0;
  int64_t id = -1;
  Point3 pos;
  std::vector<Observer> obs;
};

void MapPoint::GetNewMapPointID(int64_t& id) {
  static mtx m;
  static int64_t _id_ = 0;
  ulock<mtx> lock(m);
  id = _id_++;
}

MapPoint::MapPoint() : self(new MapPoint_impl) {}

MapPoint::MapPoint(const Point3& pos) : self(new MapPoint_impl) {
  GetNewMapPointID(self->id);
  self->pos = pos;
}

int64_t MapPoint::id() const { return self->id; }

const Point3& MapPoint::pos() const { return self->pos; }

Point3& MapPoint::pos() { return self->pos; }

const std::vector<Observer>& MapPoint::obs() const { return self->obs; }

uint64_t MapPoint::ba_times() const { return self->ba_times; }

void MapPoint::inc_ba_times() { ++self->ba_times; }

MapPoint MapPoint::create(const Point3& pos,
                          const std::vector<std::pair<int, int>>& obs,
                          sp<DB>& db) {
  MapPoint mp(pos);
  for (auto [image_i, kp_idx] : obs) {
    mp.addObserver(image_i, kp_idx);
    // update matched relationship between keypoint and mappoint
    Image& img = db->images().at(image_i);
    img.setMapptOfKpt(kp_idx, mp.id());
  }
  return mp;
}

void MapPoint::addObserver(int image_id, int kp_idx) {
  self->obs.emplace_back(image_id, kp_idx);
}

void MapPoint::clearAllObservers(sp<DB>& db) {
  auto& obs = this->self->obs;
  auto& images = db->images();
  // clear the relationship between bad point and key point
  for_each(std::execution::par, obs.begin(), obs.end(),
           [&images](const Observer& ob) {
             images.at(ob.image_id()).setMapptOfKpt(ob.kp_idx(), -1);
           });
  obs.clear();
}
