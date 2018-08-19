// Copyright sxniu 2014-9
#ifndef  INCLUDE_UI_H_
#define  INCLUDE_UI_H_

#include <vector>
#include <stdlib.h>
#include "include/UserInput.h"

template <class T>
class ImageData;

class Segmentation;

namespace ui {

struct Transpoter {
  Segmentation* seg;
  Transpoter()
    : seg(NULL) {}
};

const char* const WIN_NAME = "image";

void ShowImage(const ImageData<int>& image);
// the last element of the line_points_idx is the start point
// when line finished, line_points_idx has all line points
void DrawLine(ImageData<int>* ui_image, ImageData<int>* scr_image,
              int start_x, int start_y, int end_x, int end_y,
              int line_colour, std::vector<UserInput::LinePoint>* line_points = NULL,
              int ignore_colour = 0);
void DrawSquare(ImageData<int>* ui_image, ImageData<int>* scr_image,
                int leftup_x, int leftup_y, int rightdown_x,
                int rightdown_y, int line_colour, int ignore_colour = 0,
                std::vector<UserInput::LinePoint>* line_points = NULL);
void on_mouse(int event, int x, int y, int flags, void* param);

}  // namespace ui

#endif  // INCLUDE_UI_H_
