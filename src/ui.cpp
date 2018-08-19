// Copyright sxniu 2014-9
#include "include/ui.h"

#include <vector>
#include <stdio.h>
#include <float.h>
// #include <math.h>
// #include <assert.h>
#include <opencv/highgui.h>

#include "include/ImageData.h"
#include "include/CountTime.h"
#include "include/LazySnapping.h"
#include "include/Segmentation.h"
#include "include/UserInput.h"

using namespace cv;

namespace ui {

void ShowImage(const ImageData<int>& image) {
  int width = image.GetWidth();
  int height = image.GetHeight();
  Mat image_show(height, width, CV_8UC3);
  for (int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&image, index);
      int rgb[3] = GET_THREE_COORDINATE(colour);
      Vec3b col((uchar)rgb[2], (uchar)rgb[1], (uchar)rgb[0]);
      image_show.at<Vec3b>(y, x) = col;
    }
  }
  imshow(WIN_NAME, image_show);
}

void DrawLine(ImageData<int>* ui_image, ImageData<int>* scr_image,
              int start_x, int start_y, int end_x, int end_y,
              int line_colour, std::vector<UserInput::LinePoint>* line_points,
              int ignore_colour) {
  assert(ui_image != NULL && scr_image != NULL);
  int width = ui_image->GetWidth();
  double k_line = end_x - start_x == 0 ? (end_y - start_y > 0 ? DBL_MAX : -DBL_MAX) :
                    static_cast<double>(end_y - start_y) / (end_x - start_x);
  int x = start_x;
  int y = start_y;

  int y_dist = abs(end_y - start_y);
  int x_dist = abs(end_x - start_x);
  int times = x_dist > y_dist ? x_dist : y_dist;

  for (int j = 0; j < times; ++j) {
    if (x_dist > y_dist) {
      x = end_x - start_x > 0 ? x + 1 : x - 1;
      y = static_cast<int>((x - start_x) * k_line + start_y);
    } else {
      if (k_line == DBL_MAX) {
        ++y;
      } else if (k_line == -DBL_MAX) {
        --y;
      } else {
        y = end_y - start_y > 0 ? y + 1 : y - 1;
        x = static_cast<int>((y - start_y) / k_line + start_x);
      }
    }
    int index = y * width + x;
    int ui_col = GET_PIXEL(ui_image, index);
    if ((ui_col & RIGHT_HALF) != (line_colour & RIGHT_HALF) &&
        (ui_col & RIGHT_HALF) != (ignore_colour & RIGHT_HALF)) {
      if (line_points != NULL) {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = GET_PIXEL(scr_image, index);
        lp.direction_x = end_x - start_x;
        lp.direction_y = end_y - start_y;
        line_points->push_back(lp);
      }
      SET_PIXEL(ui_image, index, line_colour);
    }
  }
}

void DrawSquare(ImageData<int>* ui_image, ImageData<int>* scr_image,
                int leftup_x, int leftup_y, int rightdown_x, int rightdown_y,
                int line_colour, int ignore_colour,
                std::vector<UserInput::LinePoint>* line_points) {
  assert(ui_image != NULL);
  int width = ui_image->GetWidth();

  int y_leftup = leftup_y;
  int x_leftup = leftup_x;
  int y_rightdown = rightdown_y;
  int x_rightdown = rightdown_x;

  int leftup_pos = y_leftup * width + x_leftup;
  int rightdown_pos = y_rightdown * width + x_rightdown;
  int rightup_pos = y_leftup * width + x_rightdown;
  int leftdown_pos = y_rightdown * width + x_leftup;

  ui::DrawLine(ui_image, scr_image, leftup_x, leftup_y, rightdown_x, leftup_y,
               line_colour, line_points, ignore_colour);
  ui::DrawLine(ui_image, scr_image, rightdown_x, leftup_y, rightdown_x, rightdown_y,
               line_colour, line_points, ignore_colour);
  ui::DrawLine(ui_image, scr_image, rightdown_x, rightdown_y, leftup_x, rightdown_y,
               line_colour, line_points, ignore_colour);
  ui::DrawLine(ui_image, scr_image, leftup_x, rightdown_y, leftup_x, leftup_y,
               line_colour, line_points, ignore_colour);
}

void on_mouse(int event, int x, int y, int flags, void* param) {
  Transpoter* tp = reinterpret_cast<Transpoter*>(param);
  Segmentation* seg = tp->seg;
  ImageData<int>* ui_image = seg->GetUiImage();
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();
  int index = y * width + x;

  bool is_show = true;
  if (event == CV_EVENT_LBUTTONDOWN) {
    seg->DoLeftButtonDown(x, y);
  } else if (event == CV_EVENT_RBUTTONDOWN) {
    seg->DoRightButtonDown(x, y);
  } else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) {
    seg->DoLeftMouseMove(x, y);
  } else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_RBUTTON)) {
    seg->DoRightMouseMove(x, y);
  } else if (event == CV_EVENT_LBUTTONUP) {
    seg->DoLeftButtonUp(x, y);
  } else if (event == CV_EVENT_RBUTTONUP) {
    seg->DoRightButtonUp(x, y);
  } else {
    is_show = false;
  }
  if (is_show) {
    ShowImage(*ui_image);
  }
}

}  // namespace ui
