#pragma once

#include <boost/format.hpp>
#include <iostream>

#include "common_type.h"

/// \brief output msg in multithread env
/// \param fmt
inline void Print(const boost::format& fmt) {
  using namespace std;
  cout.sync_with_stdio(false);
  static mtx m;
  ulock<mtx> lock(m);
  cout << fmt;
}
