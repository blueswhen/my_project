// Copyright sxniu 2014-8
#include "include/WatershedRegion.h"
#include <vector>
#include <algorithm>
#include <assert.h>
#include "include/ImageData.h"

// class WatershedRegionGroup
WatershedRegionGroup::WatershedRegionGroup(const ImageData<int>& source_image,
                                           const ImageData<int>& marked_image,
                                           const std::vector<int>& sub_mark_index,
                                           const std::vector<int>& bck_mark_index,
                                           int region_count, int start_mark_num)
  : m_regions(std::vector<WatershedRegionInfo>(region_count))
  , m_region_num_offset(start_mark_num)
  , m_sub_area_means(std::vector<std::vector<double> >())
  , m_bck_area_means(std::vector<std::vector<double> >())
  , m_source_image(source_image)
  , m_marked_image(marked_image)
  , m_region_count(region_count) {
  int width = m_source_image.GetWidth();
  int height = m_source_image.GetHeight();

  for (int i = 0; i < sub_mark_index.size(); ++i) {
    int region_num = GET_PIXEL(&m_marked_image, sub_mark_index[i]);
    if (region_num <= 0) {
      continue;
    }
    int region_index = region_num - m_region_num_offset;
    if (m_regions[region_index].m_scene != SUBJECT) {
      m_regions[region_index].m_scene = SUBJECT;
    }
  }

  for (int i = 0; i < bck_mark_index.size(); ++i) {
    int region_num = GET_PIXEL(&m_marked_image, bck_mark_index[i]);
    if (region_num <= 0) {
      continue;
    }
    int region_index = region_num - m_region_num_offset;
    if (m_regions[region_index].m_scene != BACKGROUND) {
      m_regions[region_index].m_scene = BACKGROUND;
    }
  }

  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int index = y * width + x;
      int mark = GET_PIXEL(&m_marked_image, index);
      if (mark > 0) {
        AddRegionPoint(mark, index);
      } else if (mark == WASHED) {
        AnalyzeWatershedPoint(index);
      }
    }
  }
}

void WatershedRegionGroup::PrintRegionPointsInfo() {
  for (int i = 0; i < m_regions.size(); ++i) {
    printf("count = %d\n", m_regions[i].m_samples_count);
  }
}

int WatershedRegionGroup::GetRegionCount() const {
  return m_region_count;
}

void WatershedRegionGroup::AddRegionPoint(int region_num, int index) {
  int sample = GET_PIXEL(&m_source_image, index);
  int region_index = region_num - m_region_num_offset;
  m_regions[region_index].m_region_num = region_num;
  int rgb[3] = GET_THREE_COORDINATE(sample);
  m_regions[region_index].m_sum_col[0] += rgb[0];
  m_regions[region_index].m_sum_col[1] += rgb[1];
  m_regions[region_index].m_sum_col[2] += rgb[2];
  m_regions[region_index].m_samples_count++;
  m_regions[region_index].m_is_mean_changed = true;
}

void WatershedRegionGroup::AnalyzeWatershedPoint(int washed_index) {
  int width = m_marked_image.GetWidth();
  int height = m_marked_image.GetHeight();
  int y = washed_index / width;
  int x = washed_index -  y * width;
  int arrounds[8] = EIGHT_ARROUND_POSITION(x, y, width, height);
  std::vector<int> nums;
  for (int i = 0; i < 8; ++i) {
    int num = GET_PIXEL(&m_marked_image, arrounds[i]);
    if (num > 0) {
      nums.push_back(num);
    }
  }
  sort(nums.begin(), nums.end());
  nums.erase(unique(nums.begin(), nums.end()), nums.end());
  for (int i = nums.size() - 1; i >= 0; --i) {
    int index = nums[i] - m_region_num_offset;
    WatershedRegionInfo& region = m_regions[index];
    for (int j = i - 1; j >= 0; --j) {
      int adjacent_region_index = nums[j] - m_region_num_offset;
      WatershedRegionInfo* adj_wri = &m_regions[adjacent_region_index];
      if ((region.m_watershed_points).find(nums[j]) == region.m_watershed_points.end()) {
        region.m_adjacent_regions.push_back(adj_wri);
        std::vector<int> vec(1, washed_index);
        (region.m_watershed_points)[nums[j]] = vec;
      } else {
        (region.m_watershed_points).find(nums[j])->second.push_back(washed_index);
      }
    }
  }
}

// class WatershedRegionInfo
WatershedRegionInfo::WatershedRegionInfo()
  : m_region_num(0)
  , m_scene(UNDEFINE)
  , m_adjacent_regions(std::vector<WatershedRegionInfo*>())
  , m_watershed_points(std::map<int, std::vector<int> >())
  , m_sum_col(std::vector<int>(3, 0))
  , m_mean_col(std::vector<double>(3, 0))
  , m_samples_count(0)
  , m_is_mean_changed(true) {}

const std::vector<double>& WatershedRegionInfo::GetRegionMeanValue() {
  if (!m_is_mean_changed) {
    return m_mean_col;
  } else {
    m_mean_col[0] = static_cast<double>(m_sum_col[0]) / m_samples_count;
    m_mean_col[1] = static_cast<double>(m_sum_col[1]) / m_samples_count;
    m_mean_col[2] = static_cast<double>(m_sum_col[2]) / m_samples_count;
    m_is_mean_changed = false;
    return m_mean_col;
  }
}
