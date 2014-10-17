// Copyright sxniu 2014-10

#include "include/Lines.h"

#include "include/ImageData.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/ui.h"

Lines::Lines()
  : m_sub_line_restart(false)
  , m_bck_line_restart(false) {}

void Lines::Reset() {
  m_sub_mark_index.clear();
  m_bck_mark_index.clear();
}

void Lines::DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour) {
  SET_PIXEL(image, pos, sub_colour);
  m_sub_mark_index.push_back(pos);
}

void Lines::DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour) {
  SET_PIXEL(image, pos, bck_colour);
  m_bck_mark_index.push_back(pos);
}

void Lines::DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour) {
  if (!m_sub_line_restart && m_sub_mark_index.size() > 0) {
    ui::DrawLine(image, &m_sub_mark_index, pos, sub_colour);
  }
  SET_PIXEL(image, pos, sub_colour);
  m_sub_mark_index.push_back(pos);
  m_sub_line_restart = false;
  // RemoveLastResult();
  // DoPartition();
}

void Lines::DrawSubjectFinish() {
  m_sub_line_restart = true;
}

void Lines::DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour) {
  if (!m_bck_line_restart && m_bck_mark_index.size() > 0) {
    ui::DrawLine(image, &m_bck_mark_index, pos, bck_colour);
  }
  SET_PIXEL(image, pos, bck_colour);
  m_bck_mark_index.push_back(pos);
  m_bck_line_restart = false;
}

void Lines::DrawBackgroundFinish() {
  m_bck_line_restart = true;
}
