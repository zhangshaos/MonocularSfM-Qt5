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

  using Image_Pose6 = std::pair<int, BAVec6>;  // <image-id, pose-vec6>
  using ObserverPt2_Pose6_Mp3 =
      std::tuple<BAVec2, double*,
                 double*>;  // <observering point2, pose' ptr map point' ptr>

  std::vector<Image_Pose6> _poses;
  std::unordered_set<int> _const_pose_ids;
  std::vector<ObserverPt2_Pose6_Mp3> _obs;
  std::unordered_map<int64_t, BAVec3> _point3s;

  /// \brief create a new \class BAParam
  /// \param[in] db database of all images
  /// \param[in] map global map
  /// \param[in] used used(registered) images by system
  /// \param[in] image the image id which is used(registered) by system NOW!\n
  ///            the \p image is -1(defualt), which means optimizing all used
  ///            images, otherwise only optimizing these used images related to
  ///            \p image
  /// \param[in] constant if constant is true, the pose ready to adjust will not adjust!
  /// \return std::shard_ptr<BAParam>
  static sp<BAParam> create(const sp<DB>& db, const sp<Map>& map,
                            const sp<UsedImages>& used, int image = -1,
                            bool constant = false);

  /// \brief add unchanged pose when optimizing
  /// \param image_id
  /// \note
  /// * if pose don't be add to BA system, ingnore that pose
  void addConstPose(int image_id);

  /// \brief write adjusted data back to \a _db and \a _map
  void writeBack() const;

 private:
  mutable sp<DB> _db;
  sp<Map> _map;
  sp<UsedImages> _used_images;
  bool is_constant_pose;

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
  BAOptimizer(const sp<BAParam>& param) : _param(param) {}

  bool optimize() override;
};

#endif  // !__monocular_optimize_h__
