// Copyright sxniu 2014-9
#ifndef  INCLUDE_GRAB_CUT_H_
#define  INCLUDE_GRAB_CUT_H_

#include "include/ImageData.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"

class GrabCut :public Segmentation {
 public:
  class GrabCutData :public SegmentationData {
   friend class GrabCut;
   public:
    GrabCutData(ImageData<int>* src_img, ImageData<int>* src_img_bck,
                int sub_line_colour, int bck_line_colour)
      : SegmentationData(src_img, src_img_bck)
      , 
      , m_left_up_point(0)
      , m_right_down_point(0) {}
    virtual void Reset() {
      SegmentationData::Reset();
      ld.Reset();
      sd.Reset();
      lod.Reset();
    }

   private:
    LinesData ld;
    SquareData sd;
    LassoData lod;
  };

  enum GrabCutType {
  };

  GrabCut(GrabCutData* gcd)
    : m_gcd(gcd)
    , m_uit(SQUARE)
    , m_left_mouse_move_restart(false)
    , m_right_mouse_move_restart(false) {}
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

 private:
  bool CheckUserMark(const ImageData<int>& source_image,
                     std::vector<int>* sub_mark_index,
                     std::vector<int>* sub_mark_value,
                     std::vector<int>* bck_mark_index,
                     std::vector<int>* bck_mark_value);
  GrabCutData* m_gcd;
  UserInputType m_uit;
  bool m_left_mouse_move_restart;
  bool m_right_mouse_move_restart;
};

#endif  // INCLUDE_GRAB_CUT_H_
