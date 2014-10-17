// Copyright sxniu 2014-9
#ifndef  INCLUDE_LAZY_SNAPPING_H_
#define  INCLUDE_LAZY_SNAPPING_H_

#include "include/Segmentation.h"

class SegmentationData;
class UserInput;

template <class T>
class ImageData;

class LazySnapping :public Segmentation {
 public:
  enum LazySnappingType {
    WATERSHED,
    PIXEL
  };

  LazySnapping(SegmentationData* sd, UserInput* usr_input)
    : Segmentation(sd, usr_input)
    , m_lazy_type(PIXEL) {}
  virtual void DoPartition();
  virtual void RemoveLastResult();
  virtual ImageData<int>* GetUiImage();
  virtual void DoLeftButtonDown(int index);
  virtual void DoRightButtonDown(int index);
  virtual void DoLeftMouseMove(int index);
  virtual void DoRightMouseMove(int index);
  virtual void DoLeftButtonUp(int index);
  virtual void DoRightButtonUp(int index);

  void SetLazySnappingMethod(LazySnappingType lst);
 private:
  bool CheckUserMark(const ImageData<int>& source_image,
                     std::vector<int>* sub_mark_index,
                     std::vector<int>* sub_mark_value,
                     std::vector<int>* bck_mark_index,
                     std::vector<int>* bck_mark_value);
  LazySnappingType m_lazy_type;
};

#endif  // INCLUDE_LAZY_SNAPPING_H_
