// Copyright sxniu 2014-10
#ifndef INCLUDE_LINES_H_
#define INCLUDE_LINES_H_

#include <vector>
#include "include/UserInput.h"

template <class T>
class ImageData;

class Lines :public UserInput {
 public:
  Lines();
  virtual void Reset();
  virtual void DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour);
  virtual void DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour);
  virtual void DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour);
  virtual void DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour);
  virtual void DrawSubjectFinish();
  virtual void DrawBackgroundFinish();

 private:
  bool m_sub_line_restart;
  bool m_bck_line_restart;
  std::vector<int> m_sub_mark_index;
  std::vector<int> m_bck_mark_index;
};

#endif  // INCLUDE_LINES_H_
