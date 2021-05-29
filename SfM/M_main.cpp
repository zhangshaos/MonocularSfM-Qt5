#include <boost/format.hpp>
#include <iostream>

#include "db_init.h"
#include "system_info.h"

using Format = boost::format;

int M_main() {
  using namespace std;

  DBInit init("./db/images", "./db");
  init.runInitializing();

  auto db = init.createDB();
  //TODO: ִ��ͼ�㷨���� db ��������ͨ�Ӽ�

  // output the relationship of all images
  cout << Format(
              "************ Total images: %1% ************\n"
              "************ Image Graph Infos ************\n%2%\n") %
              db->images().size() % db->debugString();

  fs::path images_path, graph_path;
  init.saveDB(images_path, graph_path);

  cout << Format(
              "\n************ DB file saving path ************\n"
              "* images_path is [%1%]\n* graph_path is [%2%]\n"
              "*********************************************\n") %
              quoted(images_path.string()) % quoted(graph_path.string());

  return 0;
}