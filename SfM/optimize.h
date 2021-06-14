//
/// \file src/control/optimize.h
///
/// \author zxm
/// \email xingmingzhangssr@gmail
/// \version 0.1
//

#ifndef __monocular_optimize_h__
#define __monocular_optimize_h__

//#define _USE_MATH_DEFINES 1
//#include <cmath>
#include <ceres/ceres.h>

#include <vector>

#include "common_type.h"
#include "db_init.h"
#include "map.h"

/// \brief the interface of optimizing camera pose(rotation and translation)
/// and points position
struct I_Optimize {
  /// \brief run optimization
  /// \note
  /// *
  virtual bool optimize() = 0;
  virtual ~I_Optimize() {}
};

/// \brief BundleAdjust parameter,
/// use this structure to remain the outside interface of \class I_Optimize's
/// function
struct BAParam {
  template <size_t N>
  using Arr = std::array<double, N>;
  using BAVec3 = Arr<3>;
  using BAVec2 = Arr<2>;
  using BAVec6 = Arr<6>;

  // <image-id, pose-vec6>
  using Image_Pose6 = std::pair<int, BAVec6>;
  // <observering point2, pose' ptr map point' ptr>
  using ObserverPt2_Pose6_Mp3 = std::tuple<BAVec2, double*, double*>;

  std::vector<Image_Pose6> _poses;
  std::vector<ObserverPt2_Pose6_Mp3> _obs;
  std::unordered_map<int64_t, BAVec3> _point3s;
  Arr<8> _intrinsic;

  /// \brief create a new \class BAParam
  /// \param[in] db database of all images
  /// \param[in] map global map
  /// \param[in] used used(registered) images by system
  /// \param[in] image the image id which is used(registered) by system NOW!\n
  ///            the \p image is -1(defualt), which means optimizing all used
  ///            images, otherwise only optimizing these used images related to
  ///            \p image
  /// \param[in] constant if constant is true, the pose ready to adjust will not
  /// adjust!
  /// \return std::shard_ptr<BAParam>
  static sp<BAParam> create(const sp<DB>& db, const sp<Map>& map,
                            const sp<UsedImages>& used, int image = -1,
                            bool constant = false);

  /// \brief add unoptimized pose(R) when optimizing
  /// \param image_id
  /// \note
  /// * if pose don't be add to BA system, ingnore that pose
  void addConstRotation(int image_id);

  /**
   * @brief add unoptimized pose(t) when optimizing
   * @param image_id
   */
  void addConstTranslation(int image_id);

  /**
   * @brief set intrinsic could be optimized or not
   * @param optimized
   */
  void setIntrinsicOptimized(bool optimized = true);

  /// \brief write adjusted data back to \a _db and \a _map
  void writeBack() const;

 private:
  mutable sp<DB> _db;
  sp<Map> _map;
  sp<UsedImages> _used_images;

  std::unordered_set<int> _const_rotation, _const_translation;
  std::unordered_set<int64_t> _const_pt3s;
  bool is_const_intrinsic, is_const_all_poses;

  /// \brief read all \a _used_images and optimize for them
  void readFromGlobal();

  /// \brief read related \a _used_images connected to \p image_id and optimize
  /// for them \param image_id
  void readFromLocal(int image_id);

  /// \brief read optimizing data of images specified by \p used\n
  ///        this function used by \ref readFromGlobal() and \ref
  ///        readFromLocal()
  /// \param used used(registered) images by system
  void read(const UsedImages& used);

  friend class ReTriangulator;
  friend class BAOptimizer;
  friend class OptimizerWithIntrinsic;
};

class BAOptimizer : public I_Optimize {
 public:
  /// \brief cost function used by ceres
  struct CostFunction {
    double x, y;
    /// \brief constructor
    CostFunction(double _x, double _y) : x(_x), y(_y) {}

    /// \brief used by ceres
    template <class T>
    bool operator()(const T pose[6], const T pt3[3], T residual[2]) const;

    /// \brief construct...
    /// \return new ceres::CostFunction
    static ceres::CostFunction* create(double x, double y);
  };

 private:
  sp<BAParam> _param;

 public:
  /// \brief parameters to be adjusted
  /// \param param the address of parameters
  [[deprecated("use OptimizerWithIntrinsic instead")]] BAOptimizer(
      const sp<BAParam>& param)
      : _param(param) {
    assert(0 && "This class is deprecated!!!");
  }

  [[deprecated("use OptimizerWithIntrinsic instead")]] bool optimize() override;
};

class OptimizerWithIntrinsic : public I_Optimize {
 public:
  struct CostFunc {
    double x, y;
    CostFunc(double _x, double _y) : x(_x), y(_y) {}

    /**
     * @brief
     * @tparam T double or float
     * @param intrinsic fx, fy, cx, cy, k1, k2, p1, p2
     * @param pose Rcw(vec) tcw
     * @param pt3
     * @param residual
     * @return
     */
    template <class T>
    bool operator()(const T intrinsic[8], const T pose[6], const T pt3[3],
                    T residual[2]) const;

    static ceres::CostFunction* create(double x, double y);
  };

 private:
  sp<BAParam> _param;

 public:
  OptimizerWithIntrinsic(const sp<BAParam>& param) : _param(param) {}
  bool optimize() override;
};

#endif  // !__monocular_optimize_h__
