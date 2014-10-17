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

#define SUB_COL GREEN
#define BCK_COL BLUE
#define IMAGE_OUT_NAME "result.bmp"

int main(int argc, char** argv) {
  if (argc == 1) {
    printf("error: need a image\n");
    return 1;
  }
  ImageData<int> src;
  utils::ReadImage(argv[1], &src);
  ImageData<int> src_bk(src);
  ImageData<int> marked_image;

  cv::namedWindow(ui::WIN_NAME, cv::WINDOW_AUTOSIZE);

  SegmentationData sd(&src, &src_bk, SUB_COL, BCK_COL);
  Lines ln;
  Square sr;
  LazySnapping ls(&sd, &ln);
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
      // use pixel based lazy snapping 
      ls.ResetUserInput();
      ui::ShowImage(src);
      ls.SetLazySnappingMethod(LazySnapping::PIXEL);
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
    case 'f':
      // draw square
      break;
    }
  }

exit_main:
    cv::destroyWindow(ui::WIN_NAME);
    utils::SaveImage(IMAGE_OUT_NAME, src_bk);
    return 0;
}
