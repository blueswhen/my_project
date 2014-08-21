// Copyright 2014-4 sxniu
#ifndef  INCLUDE_UTILS_H_
#define  INCLUDE_UTILS_H_

#include <vector>
#include <stdlib.h>
#include <math.h>

typedef unsigned char uchar;

#define EIGHT_ARROUND_POSITION(center_x, center_y, width, height) \
{ \
  center_y * width + std::min(center_x + 1, width - 1), \
  std::min(center_y + 1, height - 1) * width + std::min(center_x + 1, width - 1), \
  std::min(center_y + 1, height - 1) * width + center_x, \
  std::min(center_y + 1, height - 1) * width + std::max(center_x - 1, 0), \
  center_y * width + std::max(center_x - 1, 0), \
  std::max(center_y - 1, 0) * width + std::max(center_x - 1, 0), \
  std::max(center_y - 1, 0) * width + center_x, \
  std::max(center_y - 1, 0) * width + std::min(center_x + 1, width - 1), \
}

#define FOUR_ARROUND_POSITION(center_x, center_y, width, height) \
{ \
  center_y * width + std::max(center_x - 1, 0), \
  center_y * width + std::min(center_x + 1, width - 1), \
  std::max(center_y - 1, 0) * width + center_x, \
  std::min(center_y + 1, height - 1) * width + center_x, \
}

#define GET_THREE_COORDINATE(colour) \
{ \
  ((colour) & RED) >> 16, \
  ((colour) & GREEN) >> 8, \
  (colour) & BLUE \
}

#define TURN_COORDINATE_TO_COLOUR(x, y, z) ((x << 16) + (y << 8) + z)

#define THREE_DIM_DIST(a, b) \
  sqrt(((a)[0] - (b)[0]) * ((a)[0] - (b)[0]) + \
       ((a)[1] - (b)[1]) * ((a)[1] - (b)[1]) + \
       ((a)[2] - (b)[2]) * ((a)[2] - (b)[2])) \

#define THREE_DIM_DIST_SQUARE(a, b) \
  (((a)[0] - (b)[0]) * ((a)[0] - (b)[0]) + \
   ((a)[1] - (b)[1]) * ((a)[1] - (b)[1]) + \
   ((a)[2] - (b)[2]) * ((a)[2] - (b)[2])) \

#define COLOUR_DIST(a, b) \
({ \
  int tmp_a[3] = GET_THREE_COORDINATE(a); \
  int tmp_b[3] = GET_THREE_COORDINATE(b); \
  THREE_DIM_DIST(tmp_a, tmp_b); \
})

#define COLOUR_DIST_SQUARE(a, b) \
({ \
  int tmp_a[3] = GET_THREE_COORDINATE(a); \
  int tmp_b[3] = GET_THREE_COORDINATE(b); \
  THREE_DIM_DIST_SQUARE(tmp_a, tmp_b); \
})

#define GET_DISTENCE(d, s) \
{ \
  abs(d[0] - s[0]), \
  abs(d[1] - s[1]), \
  abs(d[2] - s[2]), \
}

#define GET_DIFFERENCE(d, s) \
{ \
  d[0] - s[0], \
  d[1] - s[1], \
  d[2] - s[2], \
}

template <class T>
class ImageData;

class WatershedRegionGroup;

namespace utils {

void ReadImage(const char* file_name, ImageData<int>* image_data);
void TurnGray(const ImageData<int>& input_image, ImageData<uchar>* gray_image);
void SaveImage(const char* out_file_name, const ImageData<int>& image_data);
void SaveImage(const char* out_file_name, const ImageData<uchar>& image_data);
void GetGradiendMap(const ImageData<uchar>& gray_image, ImageData<uchar>* grad_image);
void MarkConnectedArea(const ImageData<uchar>& grad_image, ImageData<int>* marked_image,
                       int max_threshold, int start_mark_num, int* region_count);
void Watershed(const ImageData<uchar>& grad_image, ImageData<int>* marked_image,
               int start_gradient);
void Kmeans(const std::vector<uchar>& gray_vec,
            std::vector<int>* marked_vec,
            std::vector<double>* k_mean_set,
            int k, int iter, int random_seed);

// k_mean_colour_set (K, vector<double>(3))
void Kmeans(const std::vector<int>& colour_vec,
            std::vector<int>* marked_vec,
            std::vector<std::vector<double> >* k_mean_colour_set,
            int k, int iter, int random_seed);

template <typename T, typename U>
void Kmeans(const ImageData<T>& image, ImageData<U>* marked_image,
            int k, int iter) {
  if (marked_image->IsEmpty()) {
    marked_image->CreateEmptyImage(image.GetWidth(), image.GetHeight());
  }
  const std::vector<T>& image_vec = *(image.m_data);
  std::vector<U>* marked_vec = marked_image->m_data;
  int random_seed = image.GetRandomSeed();
  Kmeans(image_vec, marked_vec, NULL, k, iter, random_seed);
}

void ShowMarkedImage(ImageData<int>* marked_image);
void GraphCutBaseWatershed(const std::vector<std::vector<double> >& k_means_sub,
                           const std::vector<std::vector<double> >& k_means_bck,
                           WatershedRegionGroup* wrg);
void GraphCutBasePixel(const std::vector<std::vector<double> >& k_means_sub,
                       const std::vector<std::vector<double> >& k_means_bck,
                       const std::vector<int> sub_mark_index,
                       const std::vector<int> bck_mark_index,
                       const ImageData<int >& source_image,
                       ImageData<int>* marked_image);
void ExtractContourLine(const WatershedRegionGroup& wrg, ImageData<int>* source_image,
                        ImageData<int>* marked_image);
void ExtractContourLine(ImageData<int>* source_image,
                        ImageData<int>* marked_image);
void CreateSegImage(const ImageData<int>& marked_image,
                    const WatershedRegionGroup& wrg,
                    ImageData<int>* seg_image);
void ExtractMarkPoints(const ImageData<int>& mask_image,
                       const ImageData<int>& source_image,
                       int mark_colour,
                       std::vector<int>* mark_points,
                       std::vector<int>* mark_index);

}  // namespace utils

#endif  // INCLUDE_UTILS_H_
