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
  Lines(Lines* half_scale_lines);
  virtual void DrawFirstPointForSub(int x, int y);
  virtual void DrawFirstPointForBck(int x, int y);
  virtual void DrawSubjectBegin(int x, int y);
  virtual void DrawBackgroundBegin(int x, int y);
  virtual void DrawSubjectFinish(int x, int y);
  virtual void DrawBackgroundFinish(int x, int y);

 private:
  bool m_sub_line_restart;
  bool m_bck_line_restart;
};

#endif  // INCLUDE_LINES_H_
