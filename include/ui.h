// Copyright sxniu 2014-9
#ifndef  INCLUDE_UI_H_
#define  INCLUDE_UI_H_

#include <vector>
#include <stdlib.h>

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
void DrawLine(ImageData<int>* image, std::vector<int>* line_vec, int end_idx, int line_colour);
void on_mouse(int event, int x, int y, int flags, void* param);

}  // namespace ui

#endif  // INCLUDE_UI_H_
