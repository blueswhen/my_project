// Copyright sxniu 2014-10

#include "include/Lines.h"

#include <vector>
#include <assert.h>
#include <stdio.h>

#include "include/ImageData.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/ui.h"
#include "include/utils.h"

#define DRAW_FIRST_POINT_FOR_HALF_LINES(scene_mark_index) \
{ \
  UserInput* half_uip = m_hlf_uip;  \
  int half_width = width / 2; \
  int new_x = x / 2; \
  int new_y = y / 2; \
  while (half_uip != NULL) { \
    int new_index = new_y * half_width + new_x; \
    half_uip->scene_mark_index.push_back(new_index); \
    half_uip = half_uip->GetHalfScaleUserInput(); \
    half_width /= 2; \
    new_x /= 2; \
    new_y /= 2; \
  } \
}

#define DRAW_SCENE_BEGIN_FOR_HALF_LINES(scene_mark_index, scene_colour, is_restart) \
{ \
  UserInput* half_uip = m_hlf_uip; \
  SegmentationData* half_sd = half_uip->m_sd; \
  int half_width = width / 2; \
  int half_height = height / 2; \
  int new_x = x / 2; \
  int new_y = y / 2; \
  while (half_uip != NULL) { \
    half_sd = half_uip->m_sd; \
    assert(half_sd != NULL); \
    ImageData<int>* half_image = half_sd->GetSourceImage(); \
    std::vector<int>& half_scene_index = half_uip->scene_mark_index; \
    int new_index = std::min(new_y, half_height - 1) * half_width + \
                    std::min(new_x, half_width - 1); \
    if (!is_restart && half_scene_index.size() > 0) { \
      ui::DrawLine(half_image, &half_scene_index, new_index, scene_colour, half_width); \
    } \
    half_scene_index.push_back(new_index); \
    half_uip = half_uip->GetHalfScaleUserInput(); \
    half_width /= 2; \
    half_height /= 2; \
    new_x /= 2; \
    new_y /= 2; \
  } \
}

#define DRAW_SCENE_FINISH_FOR_HALF_LINES(scene_mark_index, scene_mark_value) \
{ \
  UserInput* half_uip = m_hlf_uip; \
  SegmentationData* half_sd = half_uip->m_sd; \
  while (half_uip != NULL) { \
    half_sd = half_uip->m_sd; \
    assert(half_sd != NULL); \
    ImageData<int>* half_image = half_sd->GetSourceImageBck(); \
    std::vector<int>& hlf_scene_index = half_uip->scene_mark_index; \
    std::vector<int>& hlf_scene_value = half_uip->scene_mark_value; \
    hlf_scene_value.clear(); \
    for (int i = 0; i < hlf_scene_index.size(); ++i) { \
      hlf_scene_value.push_back(GET_PIXEL(half_image, hlf_scene_index[i])); \
    } \
    half_uip = half_uip->GetHalfScaleUserInput(); \
  } \
}

Lines::Lines()
  : UserInput()
  , m_sub_line_restart(false)
  , m_bck_line_restart(false) {}

Lines::Lines(const char* file_name)
  : UserInput(file_name)
  , m_sub_line_restart(false)
  , m_bck_line_restart(false) {}

Lines::Lines(Lines* half_scale_lines)
  : UserInput(half_scale_lines)
  , m_sub_line_restart(false)
  , m_bck_line_restart(false) {}

void Lines::DrawFirstPointForSub(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int sub_colour = m_sd->GetSubjectColour();
  int width = ui_image->GetWidth();
  int index = y * width + x;

  SET_PIXEL(ui_image, index, sub_colour);
  m_sub_mark_index.push_back(index);

  DRAW_FIRST_POINT_FOR_HALF_LINES(m_sub_mark_index);
}

void Lines::DrawFirstPointForBck(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int bck_colour = m_sd->GetBackgroundColour();
  int width = ui_image->GetWidth();
  int index = y * width + x;

  SET_PIXEL(ui_image, index, bck_colour);
  m_bck_mark_index.push_back(index);

  DRAW_FIRST_POINT_FOR_HALF_LINES(m_bck_mark_index);
}

void Lines::DrawSubjectBegin(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int sub_colour = m_sd->GetSubjectColour();
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();
  int index = std::min(y, height - 1) * width + std::min(x, width - 1);

  if (!m_sub_line_restart && m_sub_mark_index.size() > 0) {
    ui::DrawLine(ui_image, &m_sub_mark_index, index, sub_colour, width);
  }

  SET_PIXEL(ui_image, index, sub_colour);
  m_sub_mark_index.push_back(index);
  m_sub_line_restart = false;

  DRAW_SCENE_BEGIN_FOR_HALF_LINES(m_sub_mark_index, sub_colour, m_sub_line_restart);
}

void Lines::DrawSubjectFinish(int x, int y) {
  m_sub_line_restart = true;
  assert(m_sd != NULL);
  ImageData<int>* image = m_sd->GetSourceImageBck();
  m_sub_mark_value.clear();
  int width = image->GetWidth();
  int height = image->GetHeight();
  FILE* file;
  std::string txt_name = m_file_name + ".txt";
  file = fopen(txt_name.c_str(), "w");
  assert(file);
  fprintf(file, "ln\n");
  for (int i = 0; i < m_sub_mark_index.size(); ++i) {
    m_sub_mark_value.push_back(GET_PIXEL(image, m_sub_mark_index[i]));
    int xy[2] = GET_XY(m_sub_mark_index[i], width);
    double x_v = static_cast<double>(xy[0]) / width;
    double y_v = static_cast<double>(xy[1]) / height;
    fprintf(file, "%f\n", x_v);
    fprintf(file, "%f\n", y_v);
  }
  fclose(file);

  DRAW_SCENE_FINISH_FOR_HALF_LINES(m_sub_mark_index, m_sub_mark_value);
}

void Lines::DrawBackgroundBegin(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int bck_colour = m_sd->GetBackgroundColour();
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();
  int index = std::min(y, height - 1) * width + std::min(x, width - 1);

  if (!m_bck_line_restart && m_bck_mark_index.size() > 0) {
    ui::DrawLine(ui_image, &m_bck_mark_index, index, bck_colour, width);
  }

  SET_PIXEL(ui_image, index, bck_colour);
  m_bck_mark_index.push_back(index);
  m_bck_line_restart = false;

  DRAW_SCENE_BEGIN_FOR_HALF_LINES(m_bck_mark_index, bck_colour, m_bck_line_restart);
}

void Lines::DrawBackgroundFinish(int x, int y) {
  m_bck_line_restart = true;
  assert(m_sd != NULL);
  ImageData<int>* image = m_sd->GetSourceImageBck();
  m_bck_mark_value.clear();
  for (int i = 0; i < m_bck_mark_index.size(); ++i) {
    m_bck_mark_value.push_back(GET_PIXEL(image, m_bck_mark_index[i]));
  }

  DRAW_SCENE_FINISH_FOR_HALF_LINES(m_sub_mark_index, m_sub_mark_value);
}
