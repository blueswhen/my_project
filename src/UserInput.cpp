// Copyright sxniu 2014-10

#include "include/UserInput.h"
#include <vector>
#include <stdlib.h>

#include "include/SegmentationData.h"

UserInput::UserInput()
  : m_sd(NULL)
  , m_sub_line_points(std::vector<LinePoint>())
  , m_bck_line_points(std::vector<LinePoint>())
  , m_hlf_uip(NULL)
  , m_usr_input_scene(SUBJECT)
  , m_is_cut(true) {}

UserInput::UserInput(const char* file_name)
  : m_sd(NULL)
  , m_sub_line_points(std::vector<LinePoint>())
  , m_bck_line_points(std::vector<LinePoint>())
  , m_hlf_uip(NULL)
  , m_usr_input_scene(SUBJECT)
  , m_file_name(file_name)
  , m_is_cut(true) {}

UserInput::UserInput(UserInput* hlf_uip)
  : m_sd(NULL)
  , m_sub_line_points(std::vector<LinePoint>())
  , m_bck_line_points(std::vector<LinePoint>())
  , m_hlf_uip(hlf_uip)
  , m_usr_input_scene(SUBJECT)
  , m_is_cut(true) {}

std::vector<UserInput::LinePoint>* UserInput::GetSubjectPoints() {
  return &m_sub_line_points;
}

std::vector<UserInput::LinePoint>* UserInput::GetBackgroundPoints() {
  return &m_bck_line_points;
}

std::string UserInput::GetImageName() {
  return m_file_name;
}

void UserInput::Reset() {
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  ImageData<int>* scr_image = m_sd->GetSourceImageBck();
  for (int i = 0; i < m_sub_line_points.size(); ++i) {
    int col = GET_PIXEL(ui_image, m_sub_line_points[i].index);
    if (col == m_sd->GetSubjectColour()) {
      SET_PIXEL(ui_image, m_sub_line_points[i].index,
                GET_PIXEL(scr_image, m_sub_line_points[i].index));
    }
  }
  for (int i = 0; i < m_bck_line_points.size(); ++i) {
    int col = GET_PIXEL(ui_image, m_bck_line_points[i].index);
    if (col == m_sd->GetBackgroundColour()) {
      SET_PIXEL(ui_image, m_bck_line_points[i].index,
                GET_PIXEL(scr_image, m_bck_line_points[i].index));
    }
  }
  m_sub_line_points.clear();
  m_bck_line_points.clear();
}

void UserInput::SetSegmentationData(SegmentationData* sd) {
  m_sd = sd;
}

UserInput* UserInput::GetHalfScaleUserInput() {
  return m_hlf_uip;
}

Scene UserInput::GetUsrInputScene() {
  return m_usr_input_scene;
}

void UserInput::SetUsrInputScene(Scene scn) {
  m_usr_input_scene = scn;
}

bool UserInput::IsCut() {
  return m_is_cut;
}

void UserInput::SetIsCut(bool is_cut) {
  m_is_cut = is_cut;
}
