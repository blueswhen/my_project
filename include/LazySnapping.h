// Copyright sxniu 2014-9
#ifndef  INCLUDE_LAZY_SNAPPING_H_
#define  INCLUDE_LAZY_SNAPPING_H_

#include "include/ImageData.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"

class LazySnapping :public Segmentation {
 public:
  class LazySnappingData :public SegmentationData {
   friend class LazySnapping;
   public:
    LazySnappingData(ImageData<int>* src_img, ImageData<int>* src_img_bck,
                     int sub_line_colour, int bck_line_colour)
      : SegmentationData(src_img, src_img_bck)
      , m_sub_line_colour(sub_line_colour)
      , m_bck_line_colour(bck_line_colour) {}
    virtual void Reset() {
      SegmentationData::Reset();
      m_sub_mark_index.clear();
      m_bck_mark_index.clear();
    }

   private:
    int m_sub_line_colour;
    int m_bck_line_colour;
    std::vector<int> m_sub_mark_index;
    std::vector<int> m_bck_mark_index;
    // WatershedRegionGroup* wrg;
  };

  enum LazySnappingType {
    WATERSHED,
    PIXEL
  };

  LazySnapping(LazySnappingData* lsd)
    : m_lsd(lsd)
    , m_uit(LINES)
    , m_left_mouse_move_restart(false)
    , m_right_mouse_move_restart(false)
    , m_lazy_type(PIXEL) {}
  virtual void DoPartition();
  virtual void RemoveLastResult();
  virtual void SetUserInputType(UserInputType uit);
  virtual UserInputType GetUserInputType();
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
  LazySnappingData* m_lsd;
  UserInputType m_uit;
  bool m_left_mouse_move_restart;
  bool m_right_mouse_move_restart;
  LazySnappingType m_lazy_type;
};

#endif  // INCLUDE_LAZY_SNAPPING_H_
