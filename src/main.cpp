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
#include "include/data.h"
#include "include/lazy_snapping.h"

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

  lazy_snapping::LazySnappingType lst = lazy_snapping::PIXEL;
  lazy_snapping::LazySnappingData lsd(&src, &src_bk, SUB_COL, BCK_COL, lst);
  cv::namedWindow(ui::WIN_NAME, cv::WINDOW_AUTOSIZE);
  data::RawData rd;
  rd.lsd = &lsd;
  cv::setMouseCallback(ui::WIN_NAME, ui::on_mouse, &rd);

  ui::ShowImage(src);

  for(;;) {
    int c = cv::waitKey(0);
    switch((char) c) {
    case '\x1b':  // esc key
      goto exit_main;
    case 'r':
      // reset
      // use pixel based lazy snapping 
      lsd.Reset();
      ui::ShowImage(src);
      lst = lazy_snapping::PIXEL;
      lsd.lazy_type = lst;
      break;
    case 's':
      // show source image
      ui::ShowImage(src);
      break;
    case 'w':
      // use watershed based lazy snapping
      lst = lazy_snapping::WATERSHED;
      lsd.lazy_type = lst;
      break;
    case 'm':
      // show marked image
      if (!lsd.marked_image->IsEmpty()) {
        ui::ShowImage(*(lsd.marked_image));
      } else {
        printf("marked image has no data\n");
      }
      break;
    }
  }

exit_main:
    cv::destroyWindow(ui::WIN_NAME);
    utils::SaveImage(IMAGE_OUT_NAME, src_bk);
    return 0;
}
