// Copyright sxniu 2014-10

#include "include/Square.h"

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
