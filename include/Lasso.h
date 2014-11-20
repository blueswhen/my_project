// Copyright sxniu 2014-10
#ifndef INCLUDE_LASSO_H_
#define INCLUDE_LASSO_H_

#include <vector>
#include <utility>

#include "include/UserInput.h"

template <class T>
class ImageData;

class Lasso :public UserInput {
 public:
  Lasso();
  virtual void DrawFirstPointForSub(int x, int y);
  virtual void DrawSubjectBegin(int x, int y);
  virtual void DrawSubjectFinish(int x, int y);
};

#endif  // INCLUDE_LASSO_H_
