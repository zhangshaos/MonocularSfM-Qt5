#define _CRT_SECURE_NO_WARNINGS 1
#define _USE_MATH_DEFINES 1
#include "system_info.h"

#include <boost/format.hpp>
#include <chrono>
#include <execution>
#include <iomanip>
#include <map>
#include <unordered_map>

//********************** SysTimeInfo **********************//
namespace Time = std::chrono;

// make time point's string in thread-safe
inline std::string StrSystemTimePoint(
    const Time::system_clock::time_point& tp) {
  auto ctp = Time::system_clock::to_time_t(tp);
  static mtx m;
  ulock<mtx> lock(m);
  return std::ctime(&ctp);
}

/// \brief statistical time infos when system running
class SystemInfo::SysTimeInfo {
  std::unordered_map<std::string,
                     std::pair<Time::system_clock::time_point, double>>
      _name_to_time;
  mutable smtx _mtx;

 public:
  void startRecord(const std::string& name) {
    ulock<smtx> lock(_mtx);
    _name_to_time[name].first = Time::system_clock::now();
  }
  void stopRecord(const std::string& name) {
    ulock<smtx> lock(_mtx);
    auto& tm = _name_to_time.at(name);
    tm.second = (double)Time::duration_cast<Time::microseconds>(
                    (Time::system_clock::now() - tm.first))
                    .count();
    tm.second *= 1e-6;  // cast to seoncds unit
  }

  std::string getRecord(const std::string& name) const {
    slock<smtx> lock(_mtx);
    auto [tp, seconds] = _name_to_time.at(name);
    auto ctp = Time::system_clock::to_time_t(tp);
    lock.unlock();
    return (boost::format("\n[%s cost [%g] seconds]\n") % std::quoted(name) %
            seconds)
        .str();
  }
  std::vector<std::string> getFullRecord() const {
    using namespace std;
    using namespace Time;
    map<system_clock::time_point, string> ranked;
    using Ranked_T = decltype(ranked);
    using SrcNode_T = decltype(_name_to_time)::value_type;
    using DestNode_T = Ranked_T::value_type;
    slock<smtx> lock(_mtx);
    transform(_name_to_time.begin(), _name_to_time.end(),
              insert_iterator<Ranked_T>(ranked, ranked.begin()),
              [this](const SrcNode_T& src) -> DestNode_T {
                return {src.second.first, getRecord(src.first)};
              });
    lock.unlock();
    vector<string> ans(ranked.size());
    transform(
        execution::par, ranked.begin(), ranked.end(), ans.begin(),
        [](Ranked_T::value_type& pr) -> string&& { return move(pr.second); });
    return ans;
  }
};
//************************ SysTimeInfo over *************************//

//*********************** SysStatistics ****************************//
class SystemInfo::SysStatistics {};
//********************** SysStatistics over *************************//

//********************** SysConfig ***********************//
struct SystemInfo::SysConfig {
  int _max_features = 8000;  //< 特征提取最多特征数
  double _sift_contrast_threshold = 0.04;
  double _sitf_sigma = 1.6;
  double _dist_ratio_in_filter_matches = 0.8;  //< 筛选特征点匹配
  int _min_matches_edge_in_create_image_graph =
      200;  //< Image Graph 中，边节点最小的匹配数量

  // 地图初始化时相关参数：
  int _min_inliers_in_2d2d_matching = 100;  //< 2D-2D 对应内点最小数量
  double _ransac_confidence_in_compute_H_E =
      0.9999;                              //< 求 H、E 时 ransac 置信度
  double _max_error_in_compute_F_E = 4.0;  //< 求 E 时误差阈值
  double _max_error_in_compute_H = 4.0;    //< 求 H 时误差阈值
  double _max_error_in_init_triangulte = 4.0;  //< 三角测量重投影误差阈值
  double _min_angle_in_init_triangulte =
      15.0 * M_PI / 180.0;  //< 三角测量角度误差阈值

  // Pnp 求解位姿相关参数
  int _min_inlers_in_pnp = 15;       //< Pnp 后 2D-3D 匹配的最小数量
  int _ransac_times_in_pnp = 10000;  //< Pnp ransac 迭代次数
  double _ransac_conifidence_in_pnp = 0.9999;  //< Pnp ransac 置信阈值
  double _max_error_in_pnp = 4.0;              //< Pnp 误差阈值

  // 后续三角测量相关参数
  double _max_error_in_triangulate = 4.0;  //< 重投影误差阈值
  double _min_anglue_in_triangulate = 15.0 * M_PI / 180.0;  //< 最小角度

  // BA 优化后，过滤地图点时的 重投影误差阈值、最小角度
  double _max_error_in_BA_filter = 2.0;
  double _min_angle_in_BA_filter = 10.0 * M_PI / 180.0;

  // Local BA 优化时，用于位姿调整的最大图片数量
  int _max_image_count_in_localBA = 5;
  int _max_found_image_count_in_BA_to_run_globalBA = 1;
};
//******************** SysConfig over *********************//

/// final \class SystemInfo constructor and destructor
SystemInfo::SystemInfo()
    : _time(new SysTimeInfo),
      _record(new SysStatistics),
      _conf(new SysConfig) {}

SystemInfo::~SystemInfo() {}

void SystemInfo::startTimeRecord(const std::string& name) {
  _time->startRecord(name);
}

void SystemInfo::stopTimeRecord(const std::string& name) {
  _time->stopRecord(name);
}

std::string SystemInfo::getTimeRecord(const std::string& name) const {
  return _time->getRecord(name);
}

std::vector<std::string> SystemInfo::getFullTimeRecord() const {
  return _time->getFullRecord();
}

int SystemInfo::max_features() const { return _conf->_max_features; }

double SystemInfo::dist_ratio_in_filter_matches() const {
  return _conf->_dist_ratio_in_filter_matches;
}

int SystemInfo::min_inliers_in_2d2d_matching() const {
  return _conf->_min_inliers_in_2d2d_matching;
}

int SystemInfo::min_matches_edge_in_create_image_graph() const {
  return _conf->_min_matches_edge_in_create_image_graph;
}

double SystemInfo::ransac_confidence_in_compute_H_E() const {
  return _conf->_ransac_confidence_in_compute_H_E;
}
double SystemInfo::max_error_in_compute_F_E() const {
  return _conf->_max_error_in_compute_F_E;
}
double SystemInfo::max_error_in_compute_H() const {
  return _conf->_max_error_in_compute_H;
}
double SystemInfo::max_error_in_init_triangulte() const {
  return _conf->_max_error_in_init_triangulte;
}
double SystemInfo::min_angle_in_init_triangulte() const {
  return _conf->_min_angle_in_init_triangulte;
}
int SystemInfo::min_inlers_in_pnp() const { return _conf->_min_inlers_in_pnp; }
double SystemInfo::max_error_in_pnp() const { return _conf->_max_error_in_pnp; }
double SystemInfo::max_error_in_triangulate() const {
  return _conf->_max_error_in_triangulate;
}
double SystemInfo::min_anglue_in_triangulate() const {
  return _conf->_min_anglue_in_triangulate;
}
double SystemInfo::max_error_in_BA_filter() const {
  return _conf->_max_error_in_BA_filter;
}
double SystemInfo::min_angle_in_BA_filter() const {
  return _conf->_min_angle_in_BA_filter;
}
int SystemInfo::max_image_count_in_localBA() const {
  return _conf->_max_image_count_in_localBA;
}
int SystemInfo::max_found_image_count_in_BA_to_run_globalBA() const {
  return _conf->_max_found_image_count_in_BA_to_run_globalBA;
}
double SystemInfo::ransac_confidence_in_pnp() const {
  return _conf->_ransac_conifidence_in_pnp;
}
int SystemInfo::ransac_times_in_pnp() const {
  return _conf->_ransac_times_in_pnp;
}