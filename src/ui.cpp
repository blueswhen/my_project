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

void DrawLine(ImageData<int>* image, std::vector<int>* line_vec, int end_idx, int line_colour) {
  int width = image->GetWidth();
  int height = image->GetHeight();
  int start_y = line_vec->back() / width;
  int start_x = line_vec->back() - start_y * width;
  int end_y = end_idx / width;
  int end_x = end_idx - end_y * width;
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
    line_vec->push_back(index);
    SET_PIXEL(image, index, line_colour);
  }
}

void on_mouse(int event, int x, int y, int flags, void* param) {
  Transpoter* tp = reinterpret_cast<Transpoter*>(param);
  Segmentation* seg = tp->seg;
  ImageData<int>* ui_image = seg->GetUiImage();
  int width = ui_image->GetWidth();
  int height = ui_image->GetHeight();
  int index = y * width + x;
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return;
  }

  bool is_show = true;
  if (event == CV_EVENT_LBUTTONDOWN) {
    seg->DoLeftButtonDown(index);
  } else if (event == CV_EVENT_RBUTTONDOWN) {
    seg->DoRightButtonDown(index);
  } else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) {
    seg->DoLeftMouseMove(index);
  } else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_RBUTTON)) {
    seg->DoRightMouseMove(index);
  } else if (event == CV_EVENT_LBUTTONUP) {
    seg->DoLeftButtonUp(index);
  } else if (event == CV_EVENT_RBUTTONUP) {
    seg->DoRightButtonUp(index);
  } else {
    is_show = false;
  }
  if (is_show) {
    ShowImage(*ui_image);
  }
}

}  // namespace ui
