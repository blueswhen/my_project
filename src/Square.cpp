// Copyright sxniu 2014-10

#include "include/Square.h"

#include <vector>
#include <utility>

#include "include/SegmentationData.h"
#include "include/ImageData.h"

Square::Square()
  : m_left_up_point(0)
  , m_right_down_point(0) {}

void Square::Reset() {
  m_left_up_point = 0;
  m_right_down_point = 0;
}

void Square::DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour) {

}

void Square::DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour) {

}

void Square::DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour) {

}

void Square::DrawSubjectFinish() {

}

void Square::DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour) {

}

void Square::DrawBackgroundFinish() {

}

std::pair<std::vector<int>, std::vector<int> > Square::GetSubjectPoints(
  const ImageData<int>& mask_image,
  const ImageData<int>& src_image,
  int sub_colour) {

}

std::pair<std::vector<int>, std::vector<int> > Square::GetBackgroundPoints(
  const ImageData<int>& mask_image,
  const ImageData<int>& src_image,
  int bck_colour) {

}

