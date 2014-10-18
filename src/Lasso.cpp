// Copyright sxniu 2014-10

#include "include/Lasso.h"

#include <vector>
#include <utility>

#include "include/ImageData.h"

void Lasso::Reset() {
  m_line_mark_index.clear();
}

void Lasso::DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour) {

}

void Lasso::DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour) {

}

void Lasso::DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour) {

}

void Lasso::DrawSubjectFinish() {

}

void Lasso::DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour) {

}

void Lasso::DrawBackgroundFinish() {

}

std::pair<std::vector<int>, std::vector<int> > Lasso::GetSubjectPoints(
  const ImageData<int>& mask_image,
  const ImageData<int>& src_image,
  int sub_colour) {

}

std::pair<std::vector<int>, std::vector<int> > Lasso::GetBackgroundPoints(
  const ImageData<int>& mask_image,
  const ImageData<int>& src_image,
  int bck_colour) {

}

