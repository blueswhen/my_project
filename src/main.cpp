// Copyright sxniu 2014-7

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <assert.h>
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

int main(int argc, char** argv) {
  if (argc == 1) {
    printf("error: need a image\n");
    return 1;
  }
  ImageData<int> src;
  utils::ReadImage(argv[1], &src);
  ImageData<int> src_bk;
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
  if (argc == 3) {
    FILE* file = NULL;
    file = fopen(argv[2], "r");
    assert(file);
    Data da(file, src);
    LazySnapping ls(&sd, &da);
    ls.DoPartition();
    // GrabCut gc(&sd, &da);
    // gc.DoPartition();
    utils::SaveImage(IMAGE_OUT_NAME, src);
    return 0;
  }
#endif

  Lines ln_25;
  Lines ln_50(&ln_25);
  // Lines ln_50;
  Lines ln(&ln_50);
  Square sr;
  LazySnapping ls(&sd, &ln);
  GrabCut gc(&sd, &ln);

  cv::namedWindow(ui::WIN_NAME, cv::WINDOW_AUTOSIZE);

  ui::Transpoter tp;
  tp.seg = &ls;
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
    utils::SaveImage(IMAGE_OUT_NAME, src);
    // utils::SaveImage(IMAGE_OUT_NAME, *sd.GetMarkedImage());
    // utils::SaveImage(IMAGE_OUT_NAME_2, *scale_50_sd.GetMarkedImage());
    // utils::SaveImage(IMAGE_OUT_NAME_3, *scale_25_sd.GetMarkedImage());
    // utils::SaveImage(IMAGE_OUT_NAME, scale_25_src);
    return 0;
}
