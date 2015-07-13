// Copyright sxniu 2014-10
#ifndef INCLUDE_DATA_H_
#define INCLUDE_DATA_H_

#include <vector>
#include <stdio.h>
#include <assert.h>

#include "include/UserInput.h"
#include "include/ImageData.h"

template <class T>
class ImageData;

class Data :public UserInput {
 public:
  Data(FILE* file, const ImageData<int>& src_image) {
    assert(file);
    int image_wdith = src_image.GetWidth();
    int image_height = src_image.GetHeight();
    char line[256];
    while (fgets(line, 256, file)) {
      double x_v = atof(line);
      int x = x_v * image_wdith;
      assert(fgets(line, 256, file));
      double y_v = atof(line);
      int y = y_v * image_height;
      int index = y * image_wdith + x;
      m_sub_mark_index.push_back(index);
      m_sub_mark_value.push_back(GET_PIXEL(&src_image, index));
    }
  }
};

#endif  // INCLUDE_DATA_H_
