#include <iostream>
#include <locale>

#include "incr_sfm.h"
#include "initialize.h"
#include "system_info.h"

int C_main() {
  using namespace std;

  try {
    constexpr char timer_name[] = "total SfM process";
    sys.startTimeRecord(timer_name);
    // config
    sys.camera_K(480, 405, 480, 270);

    // initializing....
    auto db = DB::createDB("./db/images.binary", "./db/image_graph.binary");
    Initialize init(db);
    sp<Map> g_map;
    if (!init.run(g_map)) {
      cout << "Initialize SfM Map failed...\n";
      return 0;
    }
    auto used_imgs = init.used_images();
    int init_image_id = init.init_image();

    // rebuilding...
    IncrSfM sfm(db, g_map, used_imgs, init_image_id);
    sfm.saveMap("./initial_map.ply");
    while (sfm.run()) {
      // TODO: visualization...
    }

    // over
    sfm.saveMap("./mappts.ply");
    sfm.savePoses("./poses.txt");
    size_t total = sfm.db->images().size();
    size_t used = sfm.used_images->size();
    if (total != sfm.used_images->size())
      cout << boost::format("Increasemental SfM faild...(%lld/%lld)\n") % used %
                  total;
    else
      cout << boost::format("Increasemental SfM succeed...(%lld)\n") % used;
    sys.stopTimeRecord(timer_name);
    cout << sys.getTimeRecord(timer_name);
  } catch (std::exception e) {
    puts(e.what());
    throw;
  }

  return 0;
}
