// Copyright sxniu 2014-10
#ifndef INCLUDE_SQUARE_H_
#define INCLUDE_SQUARE_H_

#include "include/UserInput.h"

class SegmentationData;

template <class T>
class ImageData;

class Square :public UserInput {
 public:
  Square();
  virtual void Reset();
  virtual void DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour);
  virtual void DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour);
  virtual void DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour);
  virtual void DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour);
  virtual void DrawSubjectFinish();
  virtual void DrawBackgroundFinish();

 private:
  int m_left_up_point;
  int m_right_down_point;
};

#endif  // INCLUDE_SQUARE_H_
