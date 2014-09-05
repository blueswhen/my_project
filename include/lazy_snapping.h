// Copyright sxniu 2014-9
#ifndef  INCLUDE_LAZY_SNAPPING_H_
#define  INCLUDE_LAZY_SNAPPING_H_

#include "include/ImageData.h"

namespace lazy_snapping {

enum LazySnappingType {
  WATERSHED,
  PIXEL
};

struct LazySnappingData {
  LazySnappingData(ImageData<int>* src_img, ImageData<int>* src_img_bck,
                   int sub_line_colour, int bck_line_colour, LazySnappingType lst)
    : source_image(src_img)
    , source_image_backup(src_img_bck)
    , marked_image(new ImageData<int>())
    , sub_line_colour(sub_line_colour)
    , bck_line_colour(bck_line_colour)
    , lazy_type(lst) {}
  ~LazySnappingData() {
    if (marked_image != NULL) {
      delete marked_image;
      marked_image = NULL;
    }
  }
  void Reset() {
    source_image->CopyImage(*source_image_backup);
    ClearMarkedImage();
    sub_mark_index.clear();
    bck_mark_index.clear();
  }
  void ClearMarkedImage() {
    if (marked_image != NULL) {
      delete marked_image;
      marked_image = NULL;
    }
    marked_image = new ImageData<int>();
  }
  ImageData<int>* source_image;
  ImageData<int>* source_image_backup;
  ImageData<int>* marked_image;
  std::vector<int> sub_mark_index;
  std::vector<int> bck_mark_index;
  int sub_line_colour;
  int bck_line_colour;
  LazySnappingType lazy_type;
  // WatershedRegionGroup* wrg;
};

void LazySnapping(LazySnappingData* lsd);
void RemoveLastResult(const ImageData<int>& source_image, ImageData<int>* result_image);

}  // namespace lazy_snapping

#endif  // INCLUDE_LAZY_SNAPPING_H_
