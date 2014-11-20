// Copyright sxniu 2014-10

#include "include/UserInput.h"
#include <utility>
#include <vector>
#include <stdlib.h>

#include "include/SegmentationData.h"

UserInput::UserInput()
  : m_sd(NULL)
  , m_sub_mark_index(new std::vector<int>())
  , m_sub_mark_value(new std::vector<int>())
  , m_bck_mark_index(new std::vector<int>())
  , m_bck_mark_value(new std::vector<int>())
  , m_hlf_uip(NULL) {}

UserInput::UserInput(UserInput* hlf_uip)
  : m_sd(NULL)
  , m_sub_mark_index(new std::vector<int>())
  , m_sub_mark_value(new std::vector<int>())
  , m_bck_mark_index(new std::vector<int>())
  , m_bck_mark_value(new std::vector<int>())
  , m_hlf_uip(hlf_uip) {}

UserInput::~UserInput() {
  delete m_sub_mark_index;
  delete m_sub_mark_value;
  delete m_bck_mark_index;
  delete m_bck_mark_value;
}

std::pair<std::vector<int>*, std::vector<int>* > UserInput::GetSubjectPoints() {
  return make_pair(m_sub_mark_index, m_sub_mark_value);
}

std::pair<std::vector<int>*, std::vector<int>* > UserInput::GetBackgroundPoints() {
  return make_pair(m_bck_mark_index, m_bck_mark_value);
}

void UserInput::Reset() {
  m_sub_mark_index->clear();
  m_sub_mark_value->clear();
  m_bck_mark_index->clear();
  m_bck_mark_value->clear();
}

void UserInput::SetSegmentationData(SegmentationData* sd) {
  m_sd = sd;
}

UserInput* UserInput::GetHalfScaleUserInput() {
  return m_hlf_uip;
}
