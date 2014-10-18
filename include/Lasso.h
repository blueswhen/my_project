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
  virtual void Reset();
  virtual void DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour);
  virtual void DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour);
  virtual void DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour);
  virtual void DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour);
  virtual void DrawSubjectFinish();
  virtual void DrawBackgroundFinish();
  virtual std::pair<std::vector<int>, std::vector<int> > GetSubjectPoints(
            const ImageData<int>& mask_image,
            const ImageData<int>& src_image,
            int sub_colour);
  virtual std::pair<std::vector<int>, std::vector<int> > GetBackgroundPoints(
            const ImageData<int>& mask_image,
            const ImageData<int>& src_image,
            int bck_colour);

 private:
  std::vector<int> m_line_mark_index;
};

#endif  // INCLUDE_LASSO_H_
