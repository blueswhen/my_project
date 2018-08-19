// Copyright sxniu 2014-10

#include "include/Square.h"

#include <vector>
#include <assert.h>

#include "include/SegmentationData.h"
#include "include/ImageData.h"
#include "include/ui.h"
#include "include/utils.h"

Square::Square()
  : UserInput()
  , m_left_up_x(0)
  , m_left_up_y(0)
  , m_right_down_x(0)
  , m_right_down_y(0) {}

Square::Square(const char* file_name)
  : UserInput(file_name)
  , m_left_up_x(0)
  , m_left_up_y(0)
  , m_right_down_x(0)
  , m_right_down_y(0) {}

void Square::SetLeftupPosition(int leftup_x, int leftup_y, int colour) {
  m_left_up_x = std::max(leftup_x, 0);
  m_left_up_y = std::max(leftup_y, 0);
  if (colour == UNDEFINE) return;

  ImageData<int>* image = m_sd->GetSourceImage();
  int width = image->GetWidth();
  int leftup_pos = m_left_up_y * width + m_left_up_x;
  SET_PIXEL(image, leftup_pos, colour);
}

void Square::SetRightdownPosition(int rightdown_x, int rightdown_y, int colour) {
  assert(m_sd != NULL);
  ImageData<int>* image = m_sd->GetSourceImage();
  int width = image->GetWidth();
  int height = image->GetHeight();
  m_right_down_y = std::min(rightdown_y, height - 1);
  m_right_down_x = std::min(rightdown_x, width - 1);
  if (colour == UNDEFINE) return;

  int rightdown_pos = m_right_down_y * width + m_right_down_x;
  SET_PIXEL(image, rightdown_pos, colour);
}

void Square::DrawFirstPointForSub(int x, int y) {
  assert(m_sd != NULL);
  SetLeftupPosition(x, y, m_sd->GetSubjectColour());
}

void Square::DrawFirstPointForBck(int x, int y) {
  assert(m_sd != NULL);
  SetLeftupPosition(x, y, m_sd->GetBackgroundColour());
}

void Square::DrawSubjectBegin(int x, int y) {
  assert(m_sd != NULL);
  SetRightdownPosition(x, y);
  m_sd->Reset();
  ui::DrawSquare(m_sd->GetSourceImage(), m_sd->GetSourceImageBck(), m_left_up_x, m_left_up_y,
                 m_right_down_x, m_right_down_y, m_sd->GetSubjectColour());
}

void Square::DrawBackgroundBegin(int x, int y) {
  assert(m_sd != NULL);
  SetRightdownPosition(x, y);
  m_sd->Reset();
  ui::DrawSquare(m_sd->GetSourceImage(), m_sd->GetSourceImageBck(), m_left_up_x, m_left_up_y,
                 m_right_down_x, m_right_down_y, m_sd->GetBackgroundColour());
}

void Square::DrawSubjectFinish(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* image = m_sd->GetSourceImageBck();
  int width = image->GetWidth();
  int height = image->GetHeight();

  int y_leftup = m_left_up_y;
  int x_leftup = m_left_up_x;
  int y_rightdown = m_right_down_y;
  int x_rightdown = m_right_down_x;

  Reset();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      if (y >= y_leftup && y < y_rightdown && x >= x_leftup && x < x_rightdown) {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = GET_PIXEL(image, index);
        m_sub_line_points.push_back(lp);
      } else {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = GET_PIXEL(image, index);
        m_bck_line_points.push_back(lp);
      }
    }
  }
#if 0
  FILE* file;
  std::string txt_name = m_file_name + "_sr.txt";
  file = fopen(txt_name.c_str(), "w");
  assert(file);
  fprintf(file, "sr\n");
  fprintf(file, "%f\n", static_cast<double>(m_left_up_x) / width);
  fprintf(file, "%f\n", static_cast<double>(m_left_up_y) / height);
  fprintf(file, "%f\n", static_cast<double>(m_right_down_x) / width);
  fprintf(file, "%f\n", static_cast<double>(m_right_down_y) / height);
  fclose(file);
#endif
}

void Square::DrawBackgroundFinish(int x, int y) {
}
