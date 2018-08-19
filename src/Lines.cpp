// Copyright sxniu 2014-10

#include "include/Lines.h"

#include <vector>
#include <assert.h>
#include <stdio.h>
#include <unordered_set>

#include "include/ImageData.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/ui.h"
#include "include/utils.h"

// used in DrawLine and DrawSquare functions
#define SUB_LINE_COL (m_sd->GetSubjectColour() + m_line_number)
#define BCK_LINE_COL (m_sd->GetBackgroundColour() + m_line_number)
#define ID_GAP 0x10000

#define DRAW_FIRST_POINT_FOR_HALF_LINES(scene_mark_index, scene_mark_value, scene_colour) \
{ \
  UserInput* half_uip = m_hlf_uip;  \
  int half_width = width / 2; \
  int new_x = x / 2; \
  int new_y = y / 2; \
  while (half_uip != NULL) { \
    SegmentationData* half_sd = half_uip->m_sd; \
    assert(half_sd != NULL); \
    ImageData<int>* half_image = half_sd->GetSourceImage(); \
    int new_index = new_y * half_width + new_x; \
    half_uip->scene_mark_index.push_back(new_index); \
    half_uip->scene_mark_value.push_back(GET_PIXEL(half_image, new_index)); \
    half_uip = half_uip->GetHalfScaleUserInput(); \
    half_width /= 2; \
    new_x /= 2; \
    new_y /= 2; \
  } \
}

#define DRAW_SCENE_BEGIN_FOR_HALF_LINES(scene_mark_index, scene_mark_value, scene_colour) \
{ \
  UserInput* half_uip = m_hlf_uip; \
  int half_width = width / 2; \
  int half_height = height / 2; \
  int new_x = x / 2; \
  int new_y = y / 2; \
  while (half_uip != NULL) { \
    SegmentationData* half_sd = half_uip->m_sd; \
    assert(half_sd != NULL); \
    ImageData<int>* half_image = half_sd->GetSourceImage(); \
    std::vector<int>& half_scene_index = half_uip->scene_mark_index; \
    std::vector<int>& half_scene_value = half_uip->scene_mark_value; \
    int new_index = std::min(new_y, half_height - 1) * half_width + \
                    std::min(new_x, half_width - 1); \
    int start_y = half_scene_index.back() / half_width; \
    int start_x = half_scene_index.back() - start_y * half_width; \
    ui::DrawLine(half_image, start_x, start_y, new_x, new_y, scene_colour, \
                 &half_scene_index, &half_scene_value); \
    half_uip = half_uip->GetHalfScaleUserInput(); \
    half_width /= 2; \
    half_height /= 2; \
    new_x /= 2; \
    new_y /= 2; \
  } \
}

int Lines::m_marked_area_id = 0;

double Lines::m_square_radius = 0.0;

Lines::Lines()
  : UserInput()
  , m_line_length(1.0)
  , m_line_number(ID_GAP) {
    m_move_points = new std::stack<std::pair<int, int>>();
  }

Lines::Lines(const char* file_name)
  : UserInput(file_name)
  , m_line_length(0.0)
  , m_line_number(ID_GAP) {
    m_move_points = new std::stack<std::pair<int, int>>();
  }

Lines::Lines(Lines* half_scale_lines)
  : UserInput(half_scale_lines)
  , m_line_length(0.0)
  , m_line_number(ID_GAP) {
    m_move_points = new std::stack<std::pair<int, int>>();
  }

Lines::~Lines() {
  if (m_move_points) {
    delete m_move_points;
    m_move_points = NULL;
  }
}

void Lines::Reset() {
  UserInput::Reset();
  m_line_length = 0.0;
}

void Lines::DrawFirstPointForSub(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int width = ui_image->GetWidth();
  int index = y * width + x;

  UserInput::LinePoint lp;
  lp.index = index;
  lp.value = GET_PIXEL(ui_image, index);
  m_sub_line_points.push_back(lp);

  // m_sub_mark_index.push_back(index);
  // m_sub_mark_value.push_back(GET_PIXEL(ui_image, index));
  // SET_PIXEL(ui_image, index, sub_colour);

  // DRAW_FIRST_POINT_FOR_HALF_LINES(m_sub_mark_index, m_sub_mark_value, sub_colour);
}

void Lines::DrawFirstPointForBck(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int width = ui_image->GetWidth();
  int index = y * width + x;

  UserInput::LinePoint lp;
  lp.index = index;
  lp.value = GET_PIXEL(ui_image, index);
  m_bck_line_points.push_back(lp);

  // m_bck_mark_index.push_back(index);
  // m_bck_mark_value.push_back(GET_PIXEL(ui_image, index));
  // SET_PIXEL(ui_image, index, bck_colour);

  // DRAW_FIRST_POINT_FOR_HALF_LINES(m_bck_mark_index, m_bck_mark_value, bck_colour);
}


void Lines::DrawSubjectBegin(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  ImageData<int>* scr_image = m_sd->GetSourceImageBck();
  ImageData<int>* marked_image = m_sd->GetMarkedImage();
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();

  int start_y = m_sub_line_points.back().index / width;
  int start_x = m_sub_line_points.back().index - start_y * width;

  int end_y = std::min(y, height - 1);
  int end_x = std::min(x, width - 1);

  int reverse_number =
    ReverseLine(ui_image, marked_image, end_x, end_y, SUB_LINE_COL, &m_sub_line_points);
  if (reverse_number == 0) {
    CollectMovePoints(x, y);
    ui::DrawLine(ui_image, scr_image, start_x, start_y, end_x, end_y,
                 SUB_LINE_COL, &m_sub_line_points, m_sd->GetBackgroundColour());
    SetIsCut(true);
  } else {
    SetIsCut(false);
  }
  UpdateSquare(ui_image, scr_image, marked_image, start_x, start_y, end_x, end_y, reverse_number);
  // DRAW_SCENE_BEGIN_FOR_HALF_LINES(m_sub_mark_index, m_sub_mark_value, sub_colour);
}

void Lines::DrawSubjectFinish(int x, int y) {
  m_line_number += ID_GAP;
  m_line_length = 0;
#if 0
  assert(m_sd != NULL);
  int width = image->GetWidth();
  int height = image->GetHeight();
  FILE* file;
  std::string txt_name = m_file_name + ".txt";
  file = fopen(txt_name.c_str(), "w");
  assert(file);
  fprintf(file, "ln\n");
  for (int i = 0; i < m_sub_mark_index.size(); ++i) {
    int xy[2] = GET_XY(m_sub_mark_index[i], width);
    double x_v = static_cast<double>(xy[0]) / width;
    double y_v = static_cast<double>(xy[1]) / height;
    fprintf(file, "%f\n", x_v);
    fprintf(file, "%f\n", y_v);
  }
  fclose(file);
#endif
}

void Lines::DrawBackgroundBegin(int x, int y) {
  assert(m_sd != NULL);
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  ImageData<int>* scr_image = m_sd->GetSourceImageBck();
  ImageData<int>* marked_image = m_sd->GetMarkedImage();
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();

  int start_y = m_bck_line_points.back().index / width;
  int start_x = m_bck_line_points.back().index - start_y * width;
  int end_y = std::min(y, height - 1);
  int end_x = std::min(x, width - 1);

  int reverse_number =
    ReverseLine(ui_image, marked_image, end_x, end_y, BCK_LINE_COL, &m_bck_line_points);
  if (reverse_number == 0) {
    CollectMovePoints(x, y);
    ui::DrawLine(ui_image, scr_image, start_x, start_y, end_x, end_y,
                 BCK_LINE_COL, &m_bck_line_points, m_sd->GetSubjectColour());
    CollectAccessedSubjectAreaIds();
    SetIsCut(true);
  } else {
    SetIsCut(false);
  }
  UpdateSquare(ui_image, scr_image, marked_image, start_x, start_y, end_x, end_y, reverse_number);
  // DRAW_SCENE_BEGIN_FOR_HALF_LINES(m_bck_mark_index, m_bck_mark_value, bck_colour);
}

void Lines::DrawBackgroundFinish(int x, int y) {
  m_line_number += ID_GAP;
  m_line_length = 0;
  // assert(m_sd != NULL);
  // ImageData<int>* image = m_sd->GetSourceImageBck();
  // m_bck_mark_value.clear();
  // for (int i = 0; i < m_bck_mark_index.size(); ++i) {
  //   m_bck_mark_value.push_back(GET_PIXEL(image, m_bck_mark_index[i]));
  // }
}

int Lines::GetMarkedAreaId() {
  return m_marked_area_id;
}

int Lines::GetSquareRadius() {
  return m_square_radius;
}

int Lines::ReverseLine(ImageData<int>* ui_image, ImageData<int>* marked_image,
                       int reverse_x, int reverse_y, int reverse_line_colour,
                       std::vector<UserInput::LinePoint>* line_points) {
  int reverse_number = 0;
  assert(ui_image != NULL && line_points != NULL);
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();
  auto reference_point = line_points->back();
  int reference_y = reference_point.index / width;
  int reference_x = reference_point.index - reference_y * width;
  int rev_direction_x = reverse_x - reference_x ;
  int rev_direction_y = reverse_y - reference_y ;
  int ref_direction_x = reference_point.direction_x;
  int ref_direction_y = reference_point.direction_y;
  int inner_product = rev_direction_x * ref_direction_x + rev_direction_y * ref_direction_y;
  if (inner_product > 0) {
    return reverse_number;
  }
  int search_radius = 5;
  int arr_index[8 * search_radius];
  int i = 0;
  for (int del_x = -search_radius; del_x < 0; ++del_x) {
    arr_index[i++] = std::max(-search_radius + reverse_y, 0) * width +
                     std::max(del_x + reverse_x, 0);
  }
  for (int del_x = 0; del_x < search_radius; ++del_x) {
    arr_index[i++] = std::max(-search_radius + reverse_y, 0) * width +
                     std::min(del_x + reverse_x, width - 1);
  }
  for (int del_y = -search_radius; del_y < 0; ++del_y) {
    arr_index[i++] = std::max(del_y + reverse_y, 0) * width +
                     std::min(search_radius + reverse_x, width - 1);
  }
  for (int del_y = 0; del_y < search_radius; ++del_y) {
    arr_index[i++] = std::min(del_y + reverse_y, height - 1) * width +
                     std::min(search_radius + reverse_x, width - 1);
  }
  for (int del_x = -search_radius; del_x < 0; ++del_x) {
    arr_index[i++] = std::min(search_radius + reverse_y, height - 1) * width +
                     std::max(del_x + reverse_x, 0);
  }
  for (int del_x = 0; del_x < search_radius; ++del_x) {
    arr_index[i++] = std::min(search_radius + reverse_y, height - 1) * width +
                     std::min(del_x + reverse_x, width - 1);
  }
  for (int del_y = -search_radius; del_y < 0; ++del_y) {
    arr_index[i++] = std::max(del_y + reverse_y, 0) * width +
                     std::max(-search_radius + reverse_x, 0);
  }
  for (int del_y = 0; del_y < search_radius; ++del_y) {
    arr_index[i++] = std::min(del_y + reverse_y, height - 1) * width +
                     std::max(-search_radius + reverse_x, 0);
  }
  for (int j = 0; j < 8 * search_radius; ++j) {
    int ui_col = GET_PIXEL(ui_image, arr_index[j]);
    if (ui_col == reverse_line_colour) {
      ++reverse_number;
      int back_point_idx = arr_index[j];
      // reverse line
#define RIT_DELT (line_points->rbegin() + search_radius)
      for (auto rit = line_points->rbegin(); rit < RIT_DELT; ++rit) {
        if (rit->index == back_point_idx) {
          return reverse_number;
        }
      }
      while (RIT_DELT->index != back_point_idx) {
        ++reverse_number;
        SET_PIXEL(ui_image, line_points->back().index, line_points->back().value);
        CheckAndRemoveAreaWhenReversing(marked_image, line_points->back().index);
        line_points->pop_back();
        assert(!line_points->empty());
      }
      return reverse_number;
    }
  }
  assert(reverse_number == 0);
  return reverse_number;
}

void Lines::UpdateSquare(ImageData<int>* ui_image, ImageData<int>* scr_image,
                         ImageData<int>* marked_image,
                         int start_x, int start_y, int end_x, int end_y,
                         int reverse_number) {
  double dist = sqrt((end_y - start_y) * (end_y - start_y) +
                     (end_x - start_x) * (end_x - start_x));
  auto square_scene = m_usr_input_scene == SUBJECT ? BACKGROUND : SUBJECT;
  auto& line_points = square_scene == SUBJECT ? m_sub_line_points : m_bck_line_points;
  int square_colour = 0;
  int ignore_colour = 0;
  if (square_scene == SUBJECT) {
    square_colour = m_sd->GetSubjectColour();
    ignore_colour = m_sd->GetBackgroundColour();
  } else {
    square_colour = m_sd->GetBackgroundColour();
    ignore_colour = m_sd->GetSubjectColour();
  }
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();
  int lu_x = width - 1;
  int lu_y = height - 1;
  int rd_x = 0;
  int rd_y = 0;
  for (int i = 0; i < line_points.size(); i++) {
    int y = line_points[i].index / width;
    int x = line_points[i].index - y * width;
    if (x < lu_x) {
      lu_x = x;
    }
    if (x > rd_x) {
      rd_x = x;
    }
    if (y < lu_y) {
      lu_y = y;
    }
    if (y > rd_y) {
      rd_y = y;
    }
  }
  // remove the last square line
  for (int i = 0; i < line_points.size(); i++) {
    int y = line_points[i].index / width;
    int x = line_points[i].index - y * width;
    if (GET_PIXEL(ui_image, line_points[i].index) == square_colour) {
      SET_PIXEL(ui_image, line_points[i].index, line_points[i].value);
    }
  }
  line_points.clear();

  bool is_break = false;
  int delt = 0;
  std::vector<UserInput::LinePoint> tmp;
  int n = 0;
  while (true) {
    n++;
    int leftup_x;
    int rightdown_x;
    int leftup_y;
    int rightdown_y;
    if (reverse_number == 0) {
      if (m_line_length < end_x / 1.5 || m_line_length < end_y / 1.5 ||
          m_line_length < (width - 1 - end_x) / 1.5 ||
          m_line_length < (height - 1 - end_y) / 1.5) {
        if (n == 1) {
          m_line_length += dist;
        } else {
          m_line_length++;
        }
      } else {
        is_break = true;
      }
    } else {
      const auto& lps = square_scene == SUBJECT ? m_bck_line_points : m_sub_line_points;
      int first_y = lps[0].index / width;
      int first_x = lps[0].index - first_y * width;
      int last_y = lps[lps.size() - 1].index / width;
      int last_x = lps[lps.size() - 1].index - last_y * width;
      double dist = sqrt((last_y - first_y) * (last_y - first_y) +
                         (last_x - first_x) * (last_x - first_x));
      if (n == 1) {
        m_line_length -= reverse_number;
        m_line_length = std::max(m_line_length, dist);
      } else {
        m_line_length--;
      }
    }

    leftup_x = std::max(static_cast<int>(end_x - 1.5 * m_line_length), 0);
    rightdown_x = std::min(static_cast<int>(end_x + 1.5 * m_line_length), width - 1);
    leftup_y = std::max(static_cast<int>(end_y - 1.5 * m_line_length), 0);
    rightdown_y = std::min(static_cast<int>(end_y + 1.5 * m_line_length), height - 1);

    ui::DrawSquare(ui_image, scr_image, leftup_x, leftup_y, rightdown_x, rightdown_y,
                   square_colour, ignore_colour, &line_points);

    for (int i = 0; i < line_points.size(); ++i) {
      int col = GET_PIXEL(marked_image, line_points[i].index);
      if ((square_scene == BACKGROUND && (col & RIGHT_HALF) != OLD_SUB) ||
          (square_scene == SUBJECT && (col & RIGHT_HALF) == OLD_SUB)) {
        tmp.push_back(line_points[i]);
      } else {
        int scr_col = GET_PIXEL(scr_image, line_points[i].index);
        SET_PIXEL(ui_image, line_points[i].index, scr_col);
      }
    }
    if (!tmp.empty() || is_break) {
      break;
    }
  }
  line_points.swap(tmp);
  m_square_radius = 1.5 * m_line_length;
}

void Lines::CollectMovePoints(int move_x, int move_y) {
  int width = m_sd->GetSourceImage()->GetWidth();
  int height = m_sd->GetSourceImage()->GetHeight();
  if (move_x < 0 || move_x > width - 1 || move_y < 0 || move_y > height - 1) {
    return;
  }
  int ui_col = GET_PIXEL(m_sd->GetSourceImage(), move_y * width + move_x);
  if ((m_move_points->empty() || move_y * width + move_x != m_move_points->top().first) &&
     ((ui_col & RIGHT_HALF) != (m_sd->GetSubjectColour() & RIGHT_HALF) &&
      (ui_col & RIGHT_HALF) != (m_sd->GetBackgroundColour() & RIGHT_HALF))) {
    m_marked_area_id += ID_GAP;
    std::pair<int, int> pr(move_y * width + move_x, m_marked_area_id);
    m_move_points->push(pr);
  }
}

void Lines::CollectAccessedSubjectAreaIds() {
  auto marked_image = m_sd->GetMarkedImage();
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  std::unordered_set<int> area_ids;
  int old = -1;
  for (int i = 0; i < m_bck_line_points.size(); ++i) {
    int index = m_bck_line_points[i].index;
    int y = index / width;
    int x = index - y * width;
    int arr[8] = EIGHT_ARROUND_POSITION(x, y, width, height);
    bool is_insert = false;
    int mark_col = GET_PIXEL(marked_image, index);
    if ((mark_col & RIGHT_HALF) == OLD_SUB) {
      is_insert = true;
    } else {
      for (int i = 0; i < 8; ++i) {
        mark_col = GET_PIXEL(marked_image, arr[i]);
        if ((mark_col & RIGHT_HALF) == OLD_SUB) {
          is_insert = true;
          break;
        }
      }
    }
    if (is_insert) {
      if (mark_col != old) {
        area_ids.insert(mark_col);
        old = mark_col;
      }
    }
  }
  for (auto it = area_ids.begin(); it != area_ids.end(); ++it) {
    UserInput::LinePoint lp;
    lp.index = -1;
    lp.value = *it;
    m_bck_line_points.push_back(lp);
  }
}

void Lines::CheckAndRemoveAreaWhenReversing(ImageData<int>* marked_image, int index) {
  if (index == m_move_points->top().first) {
    int area_id = m_move_points->top().second;
    int width = marked_image->GetWidth();
    int height = marked_image->GetHeight();
    for (int y = 0; y < height - 1; ++y) {
      for (int x = 0; x < width - 1; ++x) {
        int index = y * width + x;
        int colour = GET_PIXEL(marked_image, index);
        if (m_usr_input_scene == SUBJECT && colour == OLD_SUB + area_id) {
          SET_PIXEL(marked_image, index, IGNORED);
        }
        if (m_usr_input_scene == BACKGROUND && colour == OLD_BCK + area_id) {
          SET_PIXEL(marked_image, index, OLD_SUB + area_id);
        }
      }
    }
    // assert(area_id == m_marked_area_id);
    m_marked_area_id -= ID_GAP;
    m_move_points->pop();
  }
}
