// Copyright sxniu 2014-10
#ifndef INCLUDE_SQUARE_H_
#define INCLUDE_SQUARE_H_

#include <vector>

#include "include/UserInput.h"
#include "include/utils.h"

template <class T>
class ImageData;

class Square :public UserInput {
 public:
  Square();
  Square(const char* file_name);
  virtual void DrawFirstPointForSub(int x, int y);
  virtual void DrawSubjectBegin(int x, int y);
  virtual void DrawSubjectFinish(int x, int y);
  virtual void DrawFirstPointForBck(int x, int y);
  virtual void DrawBackgroundBegin(int x, int y);
  virtual void DrawBackgroundFinish(int x, int y);

 private:
  void SetLeftupPosition(int leftup_x, int leftup_y, int colour = UNDEFINE);
  void SetRightdownPosition(int rightdown_x, int rightdown_y, int colour = UNDEFINE);
  int m_left_up_x;
  int m_left_up_y;
  int m_right_down_x;
  int m_right_down_y;
};

#endif  // INCLUDE_SQUARE_H_
