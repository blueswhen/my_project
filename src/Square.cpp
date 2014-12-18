// Copyright sxniu 2014-10

#include "include/Square.h"

#include <vector>
#include <assert.h>

#include "include/SegmentationData.h"
#include "include/ImageData.h"
#include "include/ui.h"

Square::Square()
  : m_left_up_x(0)
  , m_left_up_y(0)
  , m_right_down_x(0)
  , m_right_down_y(0) {}

void Square::DrawFirstPointForSub(int x, int y) {
  m_left_up_x = std::max(x, 0);
  m_left_up_y = std::max(y, 0);
}

void Square::DrawSubjectBegin(int x, int y) {
  assert(m_sd != NULL);
  m_sd->Reset();

  ImageData<int>* image = m_sd->GetSourceImage();
  int sub_colour = m_sd->GetSubjectColour();

  int width = image->GetWidth();
  int height = image->GetHeight();

  int y_leftup = m_left_up_y;
  int x_leftup = m_left_up_x;
  int y_rightdown = std::min(y, height - 1);
  int x_rightdown = std::min(x, width - 1);

  int leftup_pos = y_leftup * width + x_leftup;
  int rightdown_pos = y_rightdown * width + x_rightdown;
  int rightup_pos = y_leftup * width + x_rightdown;
  int leftdown_pos = y_rightdown * width + x_leftup;

  SET_PIXEL(image, leftup_pos, sub_colour);

  std::vector<int> line;
  line.push_back(leftup_pos);
  ui::DrawLine(image, &line, rightup_pos, sub_colour, width);
  line.clear();

  line.push_back(rightup_pos);
  int index = y_rightdown * width + x_rightdown;
  ui::DrawLine(image, &line, index, sub_colour, width);
  line.clear();

  line.push_back(index);
  ui::DrawLine(image, &line, leftdown_pos, sub_colour, width);
  line.clear();

  line.push_back(leftdown_pos);
  ui::DrawLine(image, &line, leftup_pos, sub_colour, width);
  line.clear();
}

void Square::DrawSubjectFinish(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* image = m_sd->GetSourceImageBck();
  int width = image->GetWidth();
  int height = image->GetHeight();

  m_right_down_y = std::min(y, height - 1);
  m_right_down_x = std::min(x, width - 1);

  int y_leftup = m_left_up_y;
  int x_leftup = m_left_up_x;
  int y_rightdown = m_right_down_y;
  int x_rightdown = m_right_down_x;

  m_sub_mark_index.clear();
  m_sub_mark_value.clear();
  m_bck_mark_index.clear();
  m_bck_mark_value.clear();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      if (y >= y_leftup && y < y_rightdown && x >= x_leftup && x < x_rightdown) {
        m_sub_mark_index.push_back(index);
        m_sub_mark_value.push_back(GET_PIXEL(image, index));
      } else {
        m_bck_mark_index.push_back(index);
        m_bck_mark_value.push_back(GET_PIXEL(image, index));
      }
    }
  }
}
