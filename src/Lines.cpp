// Copyright sxniu 2014-10

#include "include/Lines.h"

#include <vector>
#include <utility>

#include "include/ImageData.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/ui.h"
#include "include/utils.h"

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

std::pair<std::vector<int>, std::vector<int> > Lines::GetSubjectPoints(
  const ImageData<int>& mask_image, const ImageData<int>& src_image, int sub_colour) {
  std::vector<int> sub_mark_index;
  std::vector<int> sub_mark_value;
  utils::ExtractMarkPoints(mask_image, src_image, sub_colour,
                           &sub_mark_value, &sub_mark_index);
  return std::make_pair(sub_mark_index, sub_mark_value);
}

std::pair<std::vector<int>, std::vector<int> > Lines::GetBackgroundPoints(
  const ImageData<int>& mask_image, const ImageData<int>& src_image, int bck_colour) {
  std::vector<int> bck_mark_index;
  std::vector<int> bck_mark_value;
  utils::ExtractMarkPoints(mask_image, src_image, bck_colour,
                           &bck_mark_value, &bck_mark_index);
  return std::make_pair(bck_mark_index, bck_mark_value);
}
