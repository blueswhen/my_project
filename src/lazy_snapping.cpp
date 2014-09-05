// Copyright sxniu 2014-9
#include "include/lazy_snapping.h"

#include <vector>
#include <stdio.h>
#include <float.h>
// #include <math.h>
#include <assert.h>
#include <opencv/highgui.h>

#include "include/ImageData.h"
#include "include/utils.h"
#include "include/colour.h"
#include "include/CountTime.h"
#include "include/WatershedRegion.h"

#define START_GRADIENT 10
// START_MARK_NUM must be positive number
#define START_MARK_NUM 100
#define K_NUM 100
#define ITER 10
#define CONTOUR_LINE RED 

using namespace cv;

namespace lazy_snapping {

bool CheckUserMark(const ImageData<int>& source_image,
                   std::vector<int>* sub_mark_index,
                   std::vector<int>* sub_mark_value,
                   std::vector<int>* bck_mark_index,
                   std::vector<int>* bck_mark_value) {
  if (sub_mark_index->size() == 0) {
    printf("error: no sub line mark\n");
    return false;
  }
  assert(sub_mark_value->size() == sub_mark_index->size());
  // if no bck line input, replace it with four boards of the image
  if (bck_mark_index->size() == 0) {
    int width = source_image.GetWidth();
    int height = source_image.GetHeight();
    int gap = 0;
    for (int y = gap; y < height - gap; y += height - 2 * gap) {
      for (int x = gap; x < width - gap; ++x) {
        int index = y * width + x;
        int colour = GET_PIXEL(&source_image, index);
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(colour);
      }
    }
    for (int x = gap; x < width - gap; x += width - 2 * gap - 1) {
      for (int y = gap; y < height - gap; ++y) {
        int index = y * width + x;
        int colour = GET_PIXEL(&source_image, index);
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(colour);
      }
    }
  }
  return true;
}

void LazySnapping(LazySnappingData* lsd) {
  ImageData<int>* mask_image = lsd->source_image;
  const ImageData<int>* source_image = lsd->source_image_backup;

  vector<int> sub_mark_index;
  vector<int> bck_mark_index;
  std::vector<int> sub_mark_value;
  std::vector<int> bck_mark_value;
  utils::ExtractMarkPoints(*mask_image, *source_image, lsd->sub_line_colour,
                           &sub_mark_value, &sub_mark_index);
  utils::ExtractMarkPoints(*mask_image, *source_image, lsd->bck_line_colour,
                           &bck_mark_value, &bck_mark_index);

  bool is_continue = CheckUserMark(*source_image, &sub_mark_index, &sub_mark_value,
                                   &bck_mark_index, &bck_mark_value);
  if (!is_continue) {
    printf("user mark not ready!\n");
    return;
  }

  std::vector<int> cluster_vec_sub(sub_mark_value.size());
  std::vector<int> cluster_vec_bck(bck_mark_value.size());
  std::vector<std::vector<double> > k_means_sub(K_NUM, std::vector<double>(3));
  std::vector<std::vector<double> > k_means_bck(K_NUM, std::vector<double>(3));
  utils::Kmeans(sub_mark_value, &cluster_vec_sub, &k_means_sub,
                K_NUM, ITER, mask_image->GetRandomSeed());
  utils::Kmeans(bck_mark_value, &cluster_vec_bck, &k_means_bck,
                K_NUM, ITER, mask_image->GetRandomSeed());

  lsd->ClearMarkedImage();
  ImageData<int>* marked_image = lsd->marked_image;
  if (lsd->lazy_type == WATERSHED) {
    ImageData<uchar> gray_image;
    ImageData<uchar> grad_image;
    utils::TurnGray(*source_image, &gray_image);
    utils::GetGradiendMap(gray_image, &grad_image);

    int region_count = 0;
    utils::MarkConnectedArea(grad_image, marked_image,
                             START_GRADIENT, START_MARK_NUM,
                             &region_count);

    utils::Watershed(grad_image, marked_image, START_GRADIENT);
    WatershedRegionGroup wrg(*source_image, *marked_image,
                             sub_mark_index, bck_mark_index,
                             region_count, START_MARK_NUM);
    utils::GraphCutBaseWatershed(k_means_sub, k_means_bck, &wrg);
    utils::ExtractContourLine(wrg, mask_image, marked_image);
  } else {
    utils::GraphCutBasePixel(k_means_sub, k_means_bck, sub_mark_index, bck_mark_index,
                             *source_image, marked_image);
    utils::ExtractContourLine(mask_image, marked_image);
  }
}

void RemoveLastResult(const ImageData<int>& source_image, ImageData<int>* result_image) {
  int width = source_image.GetWidth();
  int height = source_image.GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(result_image, index);
      if (colour == CONTOUR_LINE) {
        int src_col = GET_PIXEL(&source_image, index);
        SET_PIXEL(result_image, index, src_col);
      }
    }
  }
}

}  // namespace lazy_snapping
