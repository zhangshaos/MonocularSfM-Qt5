
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/format.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "common_type.h"
#include "image.h"
#include "image_graph.h"
#include "system_info.h"

// here
#include "db.h"

using Format = boost::format;

void DB::addImage(const Image& img) {
  ulock<smtx> lock(_mtx_images);
  assert(img.id() >= 0);
  _images[img.id()] = img;
}

sp<DB> DB::createDB(const fs::path& images_dir, const fs::path& graph_dir) {
  using namespace std;
  // set timer
  constexpr char timer_name[] = "read DB from files";
  sys.startTimeRecord(timer_name);

  ifstream f1(images_dir, ios_base::binary), f2(graph_dir, ios_base::binary);
  assert(f1.is_open() && f2.is_open());
  boost::archive::binary_iarchive bf1(f1), bf2(f2);
  auto db = make_shared<DB>();
  bf1 >> db->_images;
  bf2 >> db->_g;

  sys.stopTimeRecord(timer_name);
  cout << sys.getTimeRecord(timer_name) << endl;
  return db;
}

inline std::string GetFilenameFromPath(const std::string& path) {
  fs::path fs_path = path;
  return fs_path.filename().string();
}

std::string DB::debugString() const {
  using namespace std;
  ostringstream infos;
  // TODO: adopt this to multi-thead env...
  for (auto& p1 : _g._g) {
    int src_id = p1.first;
    auto& src_path = _images.at(src_id).path();
    auto& targets = p1.second;
    infos << Format("[%1%] %2%:\n") % src_id %
                 quoted(GetFilenameFromPath(src_path));
    if (targets.empty()) infos << "    [NULL]\n";
    for (auto& p2 : targets) {
      int target_id = p2.first;
      auto& target_path = _images.at(target_id).path();
      auto& target = p2.second;
      infos << Format("    [%1%-%3%] %2%\n") % target_id %
                   quoted(GetFilenameFromPath(target_path)) % target.matches;
    }
  }
  return infos.str();
}

void DB::saveSpace() {
  using namespace std;
  ulock<smtx> lock1(_mtx_images, std::defer_lock),
      lock2(_g._mtx, std::defer_lock);
  std::lock(lock1, lock2);
  // image'id => used key points' index array
  unordered_map<int, vector<int>> used_kpts_idx;
  for (auto& id_adjs : _g._g) {
    for (auto& id_node : id_adjs.second) {
      int i = id_adjs.first, j = id_node.first;
      // image-i <=> image-j 's key points match
      const auto& idx_matches = id_node.second.dmatches;
      // union old key points match(used_kpts_idx[i and j])
      // and current key points match(idxes_i, idxes_j)
      vector<int>&old_idxes_i = used_kpts_idx[i],
      &old_idxes_j = used_kpts_idx[j];
      vector<int> idxes_i, idxes_j, new_idxes_i, new_idxes_j;
      for (auto idx_pr : idx_matches) {
        idxes_i.emplace_back(idx_pr.first);
        idxes_j.emplace_back(idx_pr.second);
      }
      sort(idxes_i.begin(), idxes_i.end());
      sort(idxes_j.begin(), idxes_j.end());
      set_union(old_idxes_i.begin(), old_idxes_i.end(), idxes_i.begin(),
                idxes_i.end(), back_inserter(new_idxes_i));
      set_union(old_idxes_j.begin(), old_idxes_j.end(), idxes_j.begin(),
                idxes_j.end(), back_inserter(new_idxes_j));
      swap(new_idxes_i, old_idxes_i);
      swap(new_idxes_j, old_idxes_j);
    }
  }

  for (auto& id_image : _images) {
    // clear match-feature descriptor's content
    id_image.second.descp().release();
    const auto& used_idxes = used_kpts_idx.at(id_image.first);
    // clear image's key points (i, j)'s content
    auto SaveKeyPointsSpace = [&img = id_image.second](int left, int right) {
      for (int i = left + 1, j = right - 1; i <= j; ++i) {
        img.kpts()[i].clear();
      }
    };
    for (int i = 0, j = used_idxes.size(); i <= j; ++i) {
      int left = -1, right = -1;
      if (i == 0 || i == j) {
        if (i == 0) {
          left = -1;
          right = used_idxes[i];
        }
        if (i == j) {
          left = used_idxes[i - 1];
          right = id_image.second.kpts().size();
        }
      } else {
        left = used_idxes[i - 1];
        right = used_idxes[i];
      }
      SaveKeyPointsSpace(left, right);
    }
  }
}
