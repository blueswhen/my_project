// Copyright 2014-8 sxniu
#ifndef  INCLUDE_WATERSHEDREGION_H_
#define  INCLUDE_WATERSHEDREGION_H_

#include <vector>
#include <map>
#include "include/ImageData.h"

typedef unsigned char uchar;
// WASHED must be < 0
#define WASHED 0xff0000ff

class WatershedRegionGroup;

class WatershedRegionInfo {
 friend class WatershedRegionGroup;
 public:
  WatershedRegionInfo();
  ~WatershedRegionInfo();
  const std::vector<double>& GetRegionMeanValue();

  int m_region_num;
  Scene m_scene;
  std::vector<WatershedRegionInfo*> m_adjacent_regions;

  // Key is the region num of the adjacent_region, value is watershed points
  // of the two regions. Value is the vector<int> type.
  // All elements are index of the Watershed points of the adjacent regions
  std::map<int, std::vector<int> > m_watershed_points;

 private:
  std::vector<int> m_sum_col;
  std::vector<double> m_mean_col;
  int m_samples_count;
  bool m_is_mean_changed;
};

class WatershedRegionGroup {
 public:
  WatershedRegionGroup(const ImageData<int>& source_image,
                       const ImageData<int>& marked_image,
                       const std::vector<int>& sub_mark_index,
                       const std::vector<int>& bck_mark_index,
                       int region_count, int start_mark_num);
  ~WatershedRegionGroup();

  // fill m_adjacent_regions and m_watershed_points in class WatershedRegionInfo
  int GetRegionCount() const;
  void PrintRegionPointsInfo();

  // if you only want one copy, please use point
  std::vector<WatershedRegionInfo*>* m_regions;
  // the first region num
  int m_region_num_offset;

  std::vector<std::vector<double> > m_sub_area_means;
  std::vector<std::vector<double> > m_bck_area_means;

 private:
  void AddRegionPoint(int region_num, int index);
  void AnalyzeWatershedPoint(int washed_index);

  const ImageData<int>& m_source_image;
  const ImageData<int>& m_marked_image;
  int m_region_count;
};

#endif // INCLUDE_WATERSHEDREGION_H_
