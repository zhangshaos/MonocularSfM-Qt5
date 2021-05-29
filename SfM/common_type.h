#ifndef __monocular_common_type_h__
#define __monocular_common_type_h__

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <unordered_set>
#include <opencv2/core.hpp>

using Point2 = cv::Point2d;
using Point3 = cv::Point3d;

template <class T>
using sp = std::shared_ptr<T>;

template <class T>
using up = std::unique_ptr<T>;

template <class T>
using wp = std::weak_ptr<T>;

using UsedImages = std::unordered_set<int>;

using mtx = std::mutex;

using smtx = std::shared_mutex;

template<class T>
using ulock = std::unique_lock<T>;

template<class T>
using slock = std::shared_lock<T>;


#endif  // !__monocular_common_type_h__
