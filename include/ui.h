// Copyright sxniu 2014-9
#ifndef  INCLUDE_UI_H_
#define  INCLUDE_UI_H_

template <class T>
class ImageData;

namespace ui {

const char* const WIN_NAME = "image";

void ShowImage(const ImageData<int>& image);
void on_mouse(int event, int x, int y, int flags, void* param);

}  // namespace ui

#endif  // INCLUDE_UI_H_
