// Copyright sxniu 2014-10
#ifndef INCLUDE_DATA_H_
#define INCLUDE_DATA_H_

#include <vector>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "include/UserInput.h"
#include "include/ImageData.h"

template <class T>
class ImageData;

class Data :public UserInput {
 public:
  Data(FILE* file, const ImageData<int>& src_image) {
    assert(file);
    int image_width = src_image.GetWidth();
    int image_height = src_image.GetHeight();
    char line[256];
    if (!strcmp(fgets(line, 256, file), "ln\n")) {
      // line
      while (fgets(line, 256, file)) {
        double x_v = atof(line);
        int x = x_v * image_width;
        assert(fgets(line, 256, file));
        double y_v = atof(line);
        int y = y_v * image_height;
        int index = y * image_width + x;
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = GET_PIXEL(&src_image, index);
        m_sub_line_points.push_back(lp);
      }
    } else {
      // square
      int idx[4];
      int i = 0;
      while (fgets(line, 256, file)) {
        assert(i < 4);
        double x_v = atof(line);
        idx[i++] = x_v * image_width;
        assert(fgets(line, 256, file));
        double y_v = atof(line);
        idx[i++] = y_v * image_height;
      }
      int x_leftup = idx[0];
      int y_leftup = idx[1];
      int x_rightdown = idx[2];
      int y_rightdown = idx[3];
      for (int y = 0; y < image_height; ++y) {
        for (int x = 0; x < image_width; ++x) {
          int index = y * image_width + x;
          if (y >= y_leftup && y < y_rightdown && x >= x_leftup && x < x_rightdown) {
            UserInput::LinePoint lp;
            lp.index = index;
            lp.value = GET_PIXEL(&src_image, index);
            m_sub_line_points.push_back(lp);
          } else {
            UserInput::LinePoint lp;
            lp.index = index;
            lp.value = GET_PIXEL(&src_image, index);
            m_bck_line_points.push_back(lp);
          }
        }
      }
    }
  }
};

#endif  // INCLUDE_DATA_H_
