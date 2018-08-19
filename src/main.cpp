// Copyright sxniu 2014-7

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include <string>
#include <opencv/highgui.h>

#include "include/ImageData.h"
#include "include/utils.h"
#include "include/colour.h"
#include "include/CountTime.h"
#include "include/WatershedRegion.h"
#include "include/ui.h"
#include "include/LazySnapping.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"
#include "include/Lines.h"
#include "include/Square.h"
#include "include/Data.h"
#include "include/GrabCut.h"

#define SUB_COL GREEN
#define BCK_COL BLUE
#define IMAGE_OUT_NAME "result.bmp"
#define IMAGE_OUT_NAME_2 "result2.bmp"
#define IMAGE_OUT_NAME_3 "result3.bmp"

// #define NO_UI

void ScaleUpRegionArea(int center_x, int center_y, int factor,
                       ImageData<int>* src_image, ImageData<int>* new_image) {
  assert(new_image->IsEmpty());
  int width = src_image->GetWidth();
  int height = src_image->GetHeight();
  int offset_x = width / factor / 2;
  int offset_y = height / factor / 2;
  new_image->CreateEmptyImage(2 * width, height);
  int k = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if ((y == center_y - offset_y - 1 && x >= center_x - offset_x - 1 && x <= center_x + offset_x) ||
          (y == center_y + offset_y && x >= center_x - offset_x - 1 && x <= center_x + offset_x)|| 
          (x == center_x - offset_x - 1 && y >= center_y - offset_y - 1 && y <= center_y + offset_y) ||
          (x == center_x + offset_x && y >= center_y - offset_y - 1 && y <= center_y + offset_y)) {
        SET_PIXEL(new_image, y * 2 * width + x, GREEN);
      } else {
        int colour = GET_PIXEL(src_image, y * width + x);
        SET_PIXEL(new_image, y * 2 * width + x, colour);
      }
      if (x >= center_x - offset_x && x < center_x + offset_x &&
          y >= center_y - offset_y && y < center_y + offset_y) {
        int new_y = (k / (2 * offset_x)) * factor;
        int new_x = width + (k - (new_y / factor) * 2 * offset_x) * factor;
        k++;
        int colour = GET_PIXEL(src_image, y * width + x);
        for (int yy = 0; yy < factor; ++yy) {
          for (int xx = 0; xx < factor; ++xx) {
            SET_PIXEL(new_image, (new_y + yy) * 2 * width + new_x + xx, colour);
          }
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  if (argc == 1) {
    printf("error: need a image\n");
    return 1;
  }
  ImageData<int> src;
  utils::ReadImage(argv[1], &src);
  ImageData<int> src_bk;
  std::string file_name = utils::GetFileName(argv[1]);
#ifndef NO_UI
  if (argc == 3) {
    utils::ReadImage(argv[2], &src_bk);
  } else {
    src_bk = src;
  }
#else
  src_bk = src;
#endif
  ImageData<int> scale_50_src;
  ImageData<int> scale_25_src;

  utils::HalfScale(src_bk, &scale_50_src);
  utils::HalfScale(scale_50_src, &scale_25_src);
  ImageData<int> scale_50_src_bk(scale_50_src);
  ImageData<int> scale_25_src_bk(scale_25_src);

  SegmentationData scale_25_sd(&scale_25_src, &scale_25_src_bk, SUB_COL, BCK_COL, NULL);
  SegmentationData scale_50_sd(&scale_50_src, &scale_50_src_bk, SUB_COL, BCK_COL, &scale_25_sd);
  SegmentationData sd(&src, &src_bk, SUB_COL, BCK_COL, &scale_50_sd);

#ifdef NO_UI
  FILE* file = NULL;
  std::string txt_name = file_name + ".txt";
  // std::string txt_name = file_name + "_sr.txt";
  // printf("name = %s\n", txt_name.c_str());
  file = fopen(txt_name.c_str(), "r");
  assert(file);
  Data da(file, src);
  LazySnapping ls(&sd, &da);
  ls.DoPartition();
  // GrabCut gc(&sd, &da);
  // gc.DoPartition();
  // utils::SaveImage(IMAGE_OUT_NAME, src);
  utils::SaveImage(IMAGE_OUT_NAME, *sd.GetMarkedImage());
  fclose(file);
  return 0;
#else

  // Lines ln_25;
  // Lines ln_50(&ln_25);
  // Lines ln_50;
  Lines ln;
  // Lines ln(&ln_50);
  // Lines ln(file_name.c_str());
  Square sr(file_name.c_str());
  LazySnapping ls(&sd, &ln);
  // GrabCut gc(&sd, &ln);
  GrabCut gc(&sd, &sr);

  cv::namedWindow(ui::WIN_NAME, cv::WINDOW_AUTOSIZE);

  ui::Transpoter tp;
  tp.seg = &ls;
  // tp.seg = &gc;
  cv::setMouseCallback(ui::WIN_NAME, ui::on_mouse, &tp);

  ui::ShowImage(src);

  for(;;) {
    int c = cv::waitKey(0);
    switch((char) c) {
    case '\x1b':  // esc key
      goto exit_main;
    case 'r':
      // reset
      tp.seg->ResetUserInput();
      ui::ShowImage(src);
      break;
    case 's':
      // show source image
      ui::ShowImage(src);
      break;
    case 'w':
      // use watershed based lazy snapping
      ls.SetLazySnappingMethod(LazySnapping::WATERSHED);
      break;
    case 'm':
      // show marked image
      if (!sd.GetMarkedImage()->IsEmpty()) {
        // int center_x = 660;
        // int center_y = 535;
        // int factor = 4;
        // ImageData<int> new_image;
        // ScaleUpRegionArea(center_x, center_y, factor, sd.GetMarkedImage(), &new_image);
        // ui::ShowImage(new_image);
        ui::ShowImage(*sd.GetMarkedImage());
      } else {
        printf("marked image has no data\n");
      }
      break;
    case 'g':
      // GrabCut
      tp.seg = &gc;
      tp.seg->SetUserInput(&sr);
      break;
    case 'l':
      // LazySnapping
      tp.seg = &ls;
      tp.seg->SetUserInput(&ln);
      break;
    case 'n':
      // draw line
      tp.seg->SetUserInput(&ln);
      break;
    }
  }

exit_main:
    cv::destroyWindow(ui::WIN_NAME);
    // utils::SaveImage(IMAGE_OUT_NAME, src);
    utils::SaveImage(IMAGE_OUT_NAME, *sd.GetMarkedImage());
    // utils::SaveImage(IMAGE_OUT_NAME_2, *scale_50_sd.GetMarkedImage());
    // utils::SaveImage(IMAGE_OUT_NAME_3, *scale_25_sd.GetMarkedImage());
    // utils::SaveImage(IMAGE_OUT_NAME, scale_25_src);
    return 0;
#endif
}
