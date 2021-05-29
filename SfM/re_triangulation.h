#ifndef __monocular_re_triangulate_h__
#define __monocular_re_triangulate_h__

#include <vector>

#include "optimize.h"

/// \brief this class will be used to produce more map points after
/// map points being removed by LocalBA
class ReTriangulator
{
  sp<BAParam> _data;
public:
  ReTriangulator(const sp<BAParam>&);
  ~ReTriangulator();

  /// \brief produce new map point to map
  /// \return count of new map points 
  int run() const;
};

/// \brief triangulate new map points between \p cur_image and \p part_images
/// and insert the new images to \p map. \n
/// return the count of new map points.
/// \param map 
/// \param db 
/// \param part_images 
/// \param cur_image 
/// \return 
int CreateNewMapPoints(sp<Map>& map, sp<DB>& db, const UsedImages& part_images,
                       int cur_image);

#endif  // !__monocular_re_triangulate_h__
