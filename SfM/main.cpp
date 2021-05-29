// Main function for debug!!!

#include <iostream>
#include <limits>
#include <locale>

#define MAIN
#include "system_info.h"

extern int M_main();
extern int C_main();


int main() {
  using namespace std;
  std::locale::global(std::locale("zh_CN.utf8"));
  ios_base::sync_with_stdio(false);

  cout << "============ Welcome to use SfM ============\n"
          "1. input 'y' or 'n' to do or not create Database from local "
          "images\n";
  char ch = cin.get();
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  bool is_ready_to_create_db = ch == 'y' || ch == 'Y';

  cout << "2. input 'y' or 'n' to do or not start SfM\n";
  ch = cin.get();
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  bool is_ready_to_sfm = ch == 'y' || ch == 'Y';

  if (is_ready_to_create_db) M_main();
  if (is_ready_to_sfm) C_main();

  return 0;
}