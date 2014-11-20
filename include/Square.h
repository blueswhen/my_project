// Copyright sxniu 2014-10
#ifndef INCLUDE_SQUARE_H_
#define INCLUDE_SQUARE_H_

#include <vector>
#include <utility>

#include "include/UserInput.h"

template <class T>
class ImageData;

class Square :public UserInput {
 public:
  Square();
  virtual void DrawFirstPointForSub(int x, int y);
  virtual void DrawSubjectBegin(int x, int y);
  virtual void DrawSubjectFinish(int x, int y);

 private:
  int m_left_up_x;
  int m_left_up_y;
  int m_right_down_x;
  int m_right_down_y;
};

#endif  // INCLUDE_SQUARE_H_
