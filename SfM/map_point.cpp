#include "map_point.h"

#include <atomic>
#include <execution>

#include "db_init.h"

void MapPoint::GetNewMapPointID(int64_t& id) {
  static mtx m;
  static int64_t _id_ = 0;
  ulock<mtx> lock(m);
  id = _id_++;
}

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
