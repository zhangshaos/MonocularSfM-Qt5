#define _USE_MATH_DEFINES 1
#include "optimize.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <execution>
#include <opencv2/opencv.hpp>
#include <thread>

#include "algorithm.h"
#include "map_point_filter.h"
#include "system_info.h"

inline cv::Mat Vec3ToRmat(const double v3[3]) {
  cv::Mat1d m(3, 3);
  BAParam::BAVec3 vec3;
  std::copy_n(v3, 3, vec3.data());
  cv::Rodrigues(vec3, m);
  return m;
}
inline cv::Mat Vec3ToTmat(const double v3[3]) {
  cv::Mat1d m(3, 1);
  m << v3[0], v3[1], v3[2];
  return m;
}
inline void RmatToVec3(const cv::Mat& m, double d[3]) {
  BAParam::BAVec3 v3;
  cv::Rodrigues(m, v3);
  std::copy_n(v3.data(), 3, d);
}
inline void TmatToVec3(const cv::Mat& m, double d[3]) {
  d[0] = m.at<double>(0, 0);
  d[1] = m.at<double>(1, 0);
  d[2] = m.at<double>(2, 0);
}

inline void ConfigCeresSolveOption(int images,
                                   ceres::Solver::Options& options) {
  // settings for ceres solver!
  // these settings are copied from https://github.com/nebula-beta/MonocularSfM
  // CeresBA.cpp line 80~108

  constexpr int MAX_VIEWS_DENSE_SOLVER = 50;
  if (images < MAX_VIEWS_DENSE_SOLVER)
    options.linear_solver_type = ceres::DENSE_SCHUR;
  else
    options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  // options.minimizer_progress_to_stdout = true;  // DEBUG, report
  options.max_num_iterations = 100;
  options.num_threads = static_cast<int>(std::thread::hardware_concurrency());

  constexpr int MIN_IMAGES_BA = 10;
  if (images < MIN_IMAGES_BA) {
    options.function_tolerance *= 0.1;
    options.gradient_tolerance *= 0.1;
    options.parameter_tolerance *= 0.1;
    options.max_num_iterations *= 2;
    options.max_linear_solver_iterations = 200;
  }  // refine less images BA
}

bool BAOptimizer::optimize() {
  using namespace std;
  using Format = boost::format;
  if (_param->_poses.empty() || _param->_obs.empty() ||
      _param->_point3s.empty())
    return true;

  std::cout << "\n================== start BA optimizing ==================\n";
  constexpr char timer_name[] = "BA optimizer";
  sys.startTimeRecord(timer_name);

  ceres::Problem problem;
  ceres::LossFunction* loss_func = nullptr;  // new ceres::CauchyLoss(1.0);

  // NOTE: ceres residual-block's count is the times of all observation !!!
  for (const auto& p : _param->_obs) {
    auto& pt2 = get<0>(p);
    auto pose6 = get<1>(p);
    auto pt3 = get<2>(p);
    auto cost = CostFunction::create(pt2[0], pt2[1]);
    problem.AddResidualBlock(cost, loss_func, pose6, pt3);
  }

  // set unchanged pose data
  auto *const_rotation_param =
           new ceres::SubsetParameterization(6, vector<int>({0, 1, 2})),
       *const_translation_param =
           new ceres::SubsetParameterization(6, vector<int>({3, 4, 5}));
  for (auto& p : (_param->_poses)) {
    int image_id = p.first;
    auto& pose6 = p.second;
    if (_param->_const_rotation.count(image_id)) {
      problem.SetParameterization(pose6.data(), const_rotation_param);
    }
    if (_param->_const_translation.count(image_id)) {
      problem.SetParameterization(pose6.data(), const_translation_param);
    }
  }

  // set invariable camera intrinsic
  if (_param->is_const_intrinsic) {
    problem.SetParameterBlockConstant(_param->_intrinsic.data());
  }

  ceres::Solver::Options options;
  ConfigCeresSolveOption(static_cast<int>(_param->_poses.size()), options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  sys.stopTimeRecord(timer_name);
  std::cout << sys.getTimeRecord(timer_name)
            << "================== BA optimizer over ==================\n";
  if (summary.termination_type != ceres::CONVERGENCE) return false;
  return true;
}

sp<BAParam> BAParam::create(const sp<DB>& db, const sp<Map>& map,
                            const sp<UsedImages>& used, int image,
                            bool constant) {
  using namespace std;
  // set timer
  constexpr char timer_name[] = "read BA parameters";
  sys.startTimeRecord(timer_name);

  sp<BAParam> ptr = std::make_shared<BAParam>();
  ptr->_db = db;
  ptr->_map = map;
  ptr->_used_images = used;
  ptr->is_const_all_poses = constant;
  ptr->is_const_intrinsic = true;
  do {
    auto [fx, fy, cx, cy] = sys.camera_parameters();
    auto distort = sys.distort_parameter();
    ptr->_intrinsic[0] = fx;
    ptr->_intrinsic[1] = fy;
    ptr->_intrinsic[2] = cx;
    ptr->_intrinsic[3] = cy;
    ptr->_intrinsic[4] = distort[0];
    ptr->_intrinsic[5] = distort[1];
    ptr->_intrinsic[6] = distort[2];
    ptr->_intrinsic[7] = distort[3];
  } while (0);
  if (image < 0) {
    ptr->readFromGlobal();
  } else {
    ptr->readFromLocal(image);
  }

  sys.stopTimeRecord(timer_name);
  std::cout << sys.getTimeRecord(timer_name);
  return ptr;
}

void BAParam::addConstRotation(int image_id) {
  assert(image_id >= 0);
  _const_rotation.emplace(image_id);
}

void BAParam::addConstTranslation(int image_id) {
  assert(image_id >= 0);
  _const_translation.emplace(image_id);
}

void BAParam::setIntrinsicOptimized(bool optimized) {
  is_const_intrinsic = !optimized;
}

void BAParam::writeBack() const {
  using namespace std;
  using Format = boost::format;
  // set timer
  constexpr char timer_name[] = "write BA parameters back";
  sys.startTimeRecord(timer_name);

  auto& images = _db->images();

  // Rewrite camera intrinsic
  if (!is_const_intrinsic) {
    auto [fx, fy, cx, cy] = sys.camera_parameters();
    auto distort = sys.distort_parameter();
    double k1 = distort[0], k2 = distort[1], p1 = distort[2], p2 = distort[3];
    cout << Format( "************** Origin Camera Intrinsic ***************\n"
                    "fx: %1%, fy: %2%, cx: %3%, cy: %4%\n"
                    "k1: %5%, k2: %6%, p1: %7%, p2: %8%\n"
                    "******************************************************\n")
          % fx % fy % cx % cy % k1 % k2 % p1 % p2;
    fx = _intrinsic[0], fy = _intrinsic[1], cx = _intrinsic[2],
    cy = _intrinsic[3];
    k1 = _intrinsic[4], k2 = _intrinsic[5], p1 = _intrinsic[6],
    p2 = _intrinsic[7];
    sys.camera_K(fx, fy, cx, cy);
    sys.distort_parameter(k1, k2, p1, p2);
    cout << Format( "************** Current Camera Intrinsic **************\n"
                    "fx: %1%, fy: %2%, cx: %3%, cy: %4%\n"
                    "k1: %5%, k2: %6%, p1: %7%, p2: %8%\n"
                    "******************************************************\n")
          % fx % fy % cx % cy % k1 % k2 % p1 % p2;
  }

  // Rewrite R|t to \a _image_ids
  for (auto& p : _poses) {
    int image_id = p.first;
    auto& pose6 = p.second;
    Image& img = images.at(image_id);
    {
      assert(img.id() == image_id);
      Print(boost::format("*************** OLD image %1% ***************\nR "
                          "is\n%2%\nt is\n%3%\n") %
            image_id % img.Rcw() % img.tcw());
    }
    // Rewriting...
    img.Rcw(Vec3ToRmat(pose6.data()));
    img.tcw(Vec3ToTmat(pose6.data() + 3));
    {
      Print(boost::format("*************** NEW image %1% ***************\nR "
                          "is\n%2%\nt is\n%3%\n") %
            image_id % img.Rcw() % img.tcw());
    }
  }

  // Rewrite all mappoints(filter some bad points)
  atomic_int pt3_count = 0;
  using Id_and_V3 = decltype(_point3s)::value_type;
  for_each(
      execution::par, _point3s.begin(), _point3s.end(),
      [this, &images, &pt3_count](const Id_and_V3& pr) {
        auto& mp_id = pr.first;
        auto& mp = _map->getMapPoint(mp_id);  // old map point

        // Start to run Filter Op!
        auto& obs = mp.obs();
        vector<Point2> pt2s;
        vector<cv::Mat> projs;
        for (auto& ob : obs) {
          auto& image = images.at(ob.image_id());
          pt2s.emplace_back(image.kpts()[ob.kp_idx()].pt());
          cv::Mat1d proj;
          cv::hconcat(image.Rcw(), image.tcw(), proj);
          proj = sys.camera_K() * proj;
          projs.emplace_back(proj);
        }
        const int min_okay_reproj_count =
            std::max(static_cast<int>(obs.size()) - 1, 2);
        sp<I_MapPointFilter> depth_test(
            new ReprojDistanceFilter(10 * 50, projs, min_okay_reproj_count));
        sp<I_MapPointFilter> reproj_test(new ReprojErrorFilter(
            sys.max_error_in_BA_filter(), pt2s, projs, min_okay_reproj_count));
        Point3 new_pos(pr.second[0], pr.second[1], pr.second[2]);
        if (depth_test->isBad(new_pos) || reproj_test->isBad(new_pos)) {
          _map->removeMapPoint(mp_id, _db);  // remove the old map point
          return;
        }
        ++pt3_count;
        mp.pos() = new_pos;  // update the old map point
      });

  // BA is over!
  std::cout
      << boost::format(
             "\n********* BA HANDLE total %d pt3, LEFT %d pt3 *********\n") %
             _point3s.size() % pt3_count;
  sys.stopTimeRecord(timer_name);
  std::cout << sys.getTimeRecord(timer_name);
}

void BAParam::readFromGlobal() { read(*_used_images); }

void BAParam::readFromLocal(int image_id) {
  assert(_used_images->count(image_id));
  UsedImages used;
  used.emplace(image_id);

  auto& connected = _db->G().getAllConnected(image_id);
  for (auto& p : connected) {
    int id = p.first;
    if (_used_images->count(id)) {
      used.emplace(id);
      // limit the maximum of all Local Bundlement Adjustment images
      if (used.size() >= sys.max_image_count_in_localBA()) break;
    }  // LBA add connected image
  }    // ued ready for local images

  read(used);
}

void BAParam::read(const UsedImages& used) {
  using namespace std;
  assert(_poses.empty() && _obs.empty() && _point3s.empty());

  // 选择是否优化对应位姿
  for (int id : used) {
    if (is_const_all_poses) {
      addConstRotation(id);
      addConstTranslation(id);
    } else {
      // 根据图片被优化次数，决定是否优化该位姿
      const Image& img = _db->images().at(id);
      const int max_times = sys.max_image_optimized_times();
      if (img.R_ba_times() >= max_times) {
        addConstRotation(id);
      }
      if (img.t_ba_times() >= max_times) {
        addConstTranslation(id);
      }
    }
  }  // over

  // reserve memory for \a _poses, which makes the double-ptr(double*) to pose
  // will never be invalid when _poses increasing capacity.
  // \see line.264: \a _obs.emplace_back(...) for reason
  _poses.reserve(used.size());

  // 优化的图片为 \p used 集合
  // 优化的地图点为 used 观察到的所有地图点，且这些地图点，至少还有一个观察者在
  // used 中 优化的像素点为 上文地图点 的所有观察者

  for (int image_id : used) {
    const auto& image = _db->images().at(image_id);
    assert(image_id == image.id());

    // fill image's pose(R|t)
    BAVec6 pose;
    RmatToVec3(image.Rcw(), pose.data());
    TmatToVec3(image.tcw(), pose.data() + 3);
    _poses.emplace_back(image.id(), pose);

    // fill image's observer(key-point and map-point)
    const auto& kpts = image.kpts();
    for (auto& kpt : kpts) {
      if (kpt.id() < 0) continue;
      const auto& map_pt = _map->getMapPoint(kpt.id());

      // check intersection between \var used and \var map_pt.obs()
      auto IntersectionCount = [&map_pt, &used] {
        int count = 0;
        for (auto& ob : map_pt.obs())
          if (used.count(ob.image_id())) ++count;
        return count;
      };
      if (IntersectionCount() < 2) continue;

      // after checking, the \var map_pt is ready for adding to BA system
      if (_point3s.count(map_pt.id()) == 0) {
        const auto& pos = map_pt.pos();
        _point3s.emplace(map_pt.id(),
                         BAVec3{pos.x, pos.y, pos.z});  // fill map-points
      }
      // fill observer key-points
      auto pt2 = kpt.pt();
      _obs.emplace_back(BAVec2{pt2.x, pt2.y}, _poses.back().second.data(),
                        _point3s.at(map_pt.id()).data());
    }
  }  // all observer ready

  using Format = boost::format;
  std::cout
      << "\n========= start BA... read BA data... =========\n"
      << Format("****** poses: %1%, map points: %2%, observers: %3% ******\n") %
             _poses.size() % _point3s.size() % _obs.size();
}

// template instance...
template <class T>
bool BAOptimizer::CostFunction::operator()(const T pose[6], const T pt3[3],
                                           T residual[2]) const {
  T pp[3];
  ceres::AngleAxisRotatePoint(pose, pt3, pp);
  pp[0] += pose[3];
  pp[1] += pose[4];
  pp[2] += pose[5];
  for (int i = 0; i < 3; ++i) pp[i] /= pp[2];  // normalize...
  auto [fx, fy, cx, cy] = sys.camera_parameters();
  T u = T(fx) * pp[0] + T(cx), v = T(fy) * pp[1] + T(cy);  // pixel
  residual[0] = x - u;
  residual[1] = y - v;
  return true;
}

template <>
bool BAOptimizer::CostFunction::operator()<double>(const double pose[6],
                                                   const double pt3[3],
                                                   double residual[2]) const {
  double pp[3];
  ceres::AngleAxisRotatePoint(pose, pt3, pp);
  pp[0] += pose[3];
  pp[1] += pose[4];
  pp[2] += pose[5];
  for (int i = 0; i < 3; ++i) pp[i] /= pp[2];  // normalize...
  auto [fx, fy, cx, cy] = sys.camera_parameters();
  double u = double(fx) * pp[0] + double(cx),
         v = double(fy) * pp[1] + double(cy);  // pixel
  residual[0] = x - u;
  residual[1] = y - v;
  return true;
}
template <>
bool BAOptimizer::CostFunction::operator()<float>(const float pose[6],
                                                  const float pt3[3],
                                                  float residual[2]) const {
  float pp[3];
  ceres::AngleAxisRotatePoint(pose, pt3, pp);
  pp[0] += pose[3];
  pp[1] += pose[4];
  pp[2] += pose[5];
  for (int i = 0; i < 3; ++i) pp[i] /= pp[2];  // normalize...
  auto [fx, fy, cx, cy] = sys.camera_parameters();
  float u = float(fx) * pp[0] + float(cx),
        v = float(fy) * pp[1] + float(cy);  // pixel
  residual[0] = float(x) - u;
  residual[1] = float(y) - v;
  return true;
}

ceres::CostFunction* BAOptimizer::CostFunction::create(double x, double y) {
  // use auto diff...
  return new ceres::AutoDiffCostFunction<CostFunction, 2, 6, 3>(
      new CostFunction(x, y));
}

// template instance...
template <class T>
bool OptimizerWithIntrinsic::CostFunc::operator()(const T intrinsic[8],
                                                  const T pose[6],
                                                  const T pt3[3],
                                                  T residual[2]) const {
  T pp[3];
  ceres::AngleAxisRotatePoint(pose, pt3, pp);
  pp[0] += pose[3];
  pp[1] += pose[4];
  pp[2] += pose[5];
  for (int i = 0; i < 3; ++i) pp[i] /= pp[2];  // normalize...

  T fx = intrinsic[0], fy = intrinsic[1], cx = intrinsic[2], cy = intrinsic[3],
    k1 = intrinsic[4], k2 = intrinsic[5], p1 = intrinsic[6], p2 = intrinsic[7];

  // 归一化坐标 转 像素坐标
  T _x = pp[0], _y = pp[1];
  T r2 = _x * _x + _y * _y;
  T x_correct = _x * (T(1) + k1 * r2 + k2 * r2 * r2) + T(2) * p1 * _x * _y +
                p2 * (r2 + T(2) * _x * _x),
    y_corrent = _y * (T(1) + k1 * r2 + k2 * r2 * r2) +
                p1 * (r2 + T(2) * _y * _y) + T(2) * p2 * _x * _y;

  T u = fx * x_correct + cx, v = fy * y_corrent + cy;  // pixel

  residual[0] = x - u;
  residual[1] = y - v;
  return true;
}

template <>
bool OptimizerWithIntrinsic::CostFunc::operator()<double>(
    const double intrinsic[8], const double pose[6], const double pt3[3],
    double residual[2]) const {
  double pp[3];
  ceres::AngleAxisRotatePoint(pose, pt3, pp);
  pp[0] += pose[3];
  pp[1] += pose[4];
  pp[2] += pose[5];
  for (int i = 0; i < 3; ++i) pp[i] /= pp[2];  // normalize...

  double fx = intrinsic[0], fy = intrinsic[1], cx = intrinsic[2],
         cy = intrinsic[3], k1 = intrinsic[4], k2 = intrinsic[5],
         p1 = intrinsic[6], p2 = intrinsic[7];

  // 归一化坐标 转 像素坐标
  double _x = pp[0], _y = pp[1];
  double r2 = _x * _x + _y * _y;
  double x_correct = _x * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p1 * _x * _y +
                     p2 * (r2 + 2 * _x * _x),
         y_corrent = _y * (1 + k1 * r2 + k2 * r2 * r2) +
                     p1 * (r2 + 2 * _y * _y) + 2 * p2 * _x * _y;

  double u = fx * x_correct + cx, v = fy * y_corrent + cy;  // pixel

  residual[0] = x - u;
  residual[1] = y - v;
  return true;
}

template <>
bool OptimizerWithIntrinsic::CostFunc::operator()<float>(
    const float intrinsic[8], const float pose[6], const float pt3[3],
    float residual[2]) const {
  float pp[3];
  ceres::AngleAxisRotatePoint(pose, pt3, pp);
  pp[0] += pose[3];
  pp[1] += pose[4];
  pp[2] += pose[5];
  for (int i = 0; i < 3; ++i) pp[i] /= pp[2];  // normalize...

  float fx = intrinsic[0], fy = intrinsic[1], cx = intrinsic[2],
        cy = intrinsic[3], k1 = intrinsic[4], k2 = intrinsic[5],
        p1 = intrinsic[6], p2 = intrinsic[7];

  // 归一化坐标 转 像素坐标
  float _x = pp[0], _y = pp[1];
  float r2 = _x * _x + _y * _y;
  float x_correct = _x * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p1 * _x * _y +
                    p2 * (r2 + 2 * _x * _x),
        y_corrent = _y * (1 + k1 * r2 + k2 * r2 * r2) +
                    p1 * (r2 + 2 * _y * _y) + 2 * p2 * _x * _y;

  float u = fx * x_correct + cx, v = fy * y_corrent + cy;  // pixel

  residual[0] = x - u;
  residual[1] = y - v;
  return true;
}

ceres::CostFunction* OptimizerWithIntrinsic::CostFunc::create(double x,
                                                              double y) {
  return new ceres::AutoDiffCostFunction<CostFunc, 2, 8, 6, 3>(
      new CostFunc(x, y));
}

bool OptimizerWithIntrinsic::optimize() {
  using namespace std;
  using Format = boost::format;
  if (_param->_poses.empty() || _param->_obs.empty() ||
      _param->_point3s.empty())
    return true;

  std::cout << "\n================== start BA optimizing ==================\n";
  constexpr char timer_name[] = "BA optimizer";
  sys.startTimeRecord(timer_name);

  ceres::Problem problem;
  ceres::LossFunction* loss_func = nullptr;  // new ceres::CauchyLoss(1.0);

  // NOTE: ceres residual-block's count is the times of all observation !!!
  for (const auto& p : _param->_obs) {
    auto& pt2 = get<0>(p);
    auto pose6 = get<1>(p);
    auto pt3 = get<2>(p);
    auto cost = CostFunc::create(pt2[0], pt2[1]);
    problem.AddResidualBlock(cost, loss_func, _param->_intrinsic.data(), pose6,
                             pt3);
  }

  // set unchanged pose data
  auto *const_rotation_param =
           new ceres::SubsetParameterization(6, vector<int>({0, 1, 2})),
       *const_translation_param =
           new ceres::SubsetParameterization(6, vector<int>({3, 4, 5}));
  for (auto& p : (_param->_poses)) {
    int image_id = p.first;
    auto& pose6 = p.second;
    if (_param->_const_rotation.count(image_id)) {
      problem.SetParameterization(pose6.data(), const_rotation_param);
    }
    if (_param->_const_translation.count(image_id)) {
      problem.SetParameterization(pose6.data(), const_translation_param);
    }
  }

  // set invariable camera intrinsic
  if (_param->is_const_intrinsic) {
    problem.SetParameterBlockConstant(_param->_intrinsic.data());
  }

  ceres::Solver::Options options;
  ConfigCeresSolveOption(static_cast<int>(_param->_poses.size()), options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  sys.stopTimeRecord(timer_name);
  std::cout << sys.getTimeRecord(timer_name)
            << "================== BA optimizer over ==================\n";
  if (summary.termination_type != ceres::CONVERGENCE) return false;
  return true;
}
