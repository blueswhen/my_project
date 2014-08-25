// Copyright sxniu 2014-7

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <assert.h>

#include "include/ImageData.h"
#include "include/utils.h"
#include "include/colour.h"
#include "include/CountTime.h"
#include "include/WatershedRegion.h"
#include <opencv/highgui.h>

#define IMAGE_OUT_NAME "result.bmp"
#define START_GRADIENT 10
// START_MARK_NUM must be positive number
#define START_MARK_NUM 100
#define K_NUM 100
#define ITER 10
#define SUB_COL GREEN
#define BCK_COL BLUE

#define WIN_NAME "image"
#define CONTOUR_LINE RED 
using namespace cv;

enum LazySnappingType {
  WATERSHED,
  PIXEL
};

struct LazySnappingData {
  LazySnappingData(ImageData<int>* src_img, ImageData<int>* src_img_bck,
                   int sub_line_colour, int bck_line_colour, LazySnappingType lst)
    : source_image(src_img)
    , source_image_backup(src_img_bck)
    , marked_image(new ImageData<int>())
    , sub_line_colour(sub_line_colour)
    , bck_line_colour(bck_line_colour)
    , lazy_type(lst) {}
  ~LazySnappingData() {
    if (marked_image != NULL) {
      delete marked_image;
      marked_image = NULL;
    }
  }
  void Reset() {
    source_image->CopyImage(*source_image_backup);
    ClearMarkedImage();
    sub_mark_index.clear();
    bck_mark_index.clear();
  }
  void ClearMarkedImage() {
    if (marked_image != NULL) {
      delete marked_image;
      marked_image = NULL;
    }
    marked_image = new ImageData<int>();
  }
  ImageData<int>* source_image;
  ImageData<int>* source_image_backup;
  ImageData<int>* marked_image;
  std::vector<int> sub_mark_index;
  std::vector<int> bck_mark_index;
  int sub_line_colour;
  int bck_line_colour;
  LazySnappingType lazy_type;
  // WatershedRegionGroup* wrg;
};

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

void Line(ImageData<int>* image, std::vector<int>* line_vec, int end_idx, int line_colour) {
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

bool CheckUserMark(const ImageData<int>& source_image,
                   std::vector<int>* sub_mark_index,
                   std::vector<int>* sub_mark_value,
                   std::vector<int>* bck_mark_index,
                   std::vector<int>* bck_mark_value) {
  if (sub_mark_index->size() == 0) {
    printf("error: no sub line mark\n");
    return false;
  }
  assert(sub_mark_value->size() == sub_mark_index->size());
  // if no bck line input, replace it with four boards of the image
  if (bck_mark_index->size() == 0) {
    int width = source_image.GetWidth();
    int height = source_image.GetHeight();
    int gap = 0;
    for (int y = gap; y < height - gap; y += height - 2 * gap) {
      for (int x = gap; x < width - gap; ++x) {
        int index = y * width + x;
        int colour = GET_PIXEL(&source_image, index);
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(colour);
      }
    }
    for (int x = gap; x < width - gap; x += width - 2 * gap - 1) {
      for (int y = gap; y < height - gap; ++y) {
        int index = y * width + x;
        int colour = GET_PIXEL(&source_image, index);
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(colour);
      }
    }
  }
  return true;
}

void LazySnapping(LazySnappingData* lsd) {
  ImageData<int>* mask_image = lsd->source_image;
  const ImageData<int>* source_image = lsd->source_image_backup;

  vector<int> sub_mark_index;
  vector<int> bck_mark_index;
  std::vector<int> sub_mark_value;
  std::vector<int> bck_mark_value;
  utils::ExtractMarkPoints(*mask_image, *source_image, lsd->sub_line_colour,
                           &sub_mark_value, &sub_mark_index);
  utils::ExtractMarkPoints(*mask_image, *source_image, lsd->bck_line_colour,
                           &bck_mark_value, &bck_mark_index);

  bool is_continue = CheckUserMark(*source_image, &sub_mark_index, &sub_mark_value,
                                   &bck_mark_index, &bck_mark_value);
  if (!is_continue) {
    printf("user mark not ready!\n");
    return;
  }

  std::vector<int> cluster_vec_sub(sub_mark_value.size());
  std::vector<int> cluster_vec_bck(bck_mark_value.size());
  std::vector<std::vector<double> > k_means_sub(K_NUM, std::vector<double>(3));
  std::vector<std::vector<double> > k_means_bck(K_NUM, std::vector<double>(3));
  utils::Kmeans(sub_mark_value, &cluster_vec_sub, &k_means_sub,
                K_NUM, ITER, mask_image->GetRandomSeed());
  utils::Kmeans(bck_mark_value, &cluster_vec_bck, &k_means_bck,
                K_NUM, ITER, mask_image->GetRandomSeed());

  lsd->ClearMarkedImage();
  ImageData<int>* marked_image = lsd->marked_image;
  if (lsd->lazy_type == WATERSHED) {
    ImageData<uchar> gray_image;
    ImageData<uchar> grad_image;
    utils::TurnGray(*source_image, &gray_image);
    utils::GetGradiendMap(gray_image, &grad_image);

    int region_count = 0;
    utils::MarkConnectedArea(grad_image, marked_image,
                             START_GRADIENT, START_MARK_NUM,
                             &region_count);

    utils::Watershed(grad_image, marked_image, START_GRADIENT);
    WatershedRegionGroup wrg(*source_image, *marked_image,
                             sub_mark_index, bck_mark_index,
                             region_count, START_MARK_NUM);
    utils::GraphCutBaseWatershed(k_means_sub, k_means_bck, &wrg);
    utils::ExtractContourLine(wrg, mask_image, marked_image);
  } else {
    utils::GraphCutBasePixel(k_means_sub, k_means_bck, sub_mark_index, bck_mark_index,
                             *source_image, marked_image);
    utils::ExtractContourLine(mask_image, marked_image);
  }
}

void RemoveLastResult(const ImageData<int>& source_image, ImageData<int>* result_image) {
  int width = source_image.GetWidth();
  int height = source_image.GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(result_image, index);
      if (colour == CONTOUR_LINE) {
        int src_col = GET_PIXEL(&source_image, index);
        SET_PIXEL(result_image, index, src_col);
      }
    }
  }
}

bool g_sub_restart = false;
bool g_bck_restart = false;

void on_mouse(int event, int x, int y, int flags, void* param) {
  LazySnappingData* lsd = reinterpret_cast<LazySnappingData*>(param);
  ImageData<int>* image = lsd->source_image;
  std::vector<int>& sub_line_vec = lsd->sub_mark_index;
  std::vector<int>& bck_line_vec = lsd->bck_mark_index;
  int width = image->GetWidth();
  int height = image->GetHeight();
  int index = y * width + x;
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return;
  }

  bool is_show = true;
  if (event == CV_EVENT_LBUTTONDOWN) {
    SET_PIXEL(image, index, lsd->sub_line_colour);
    sub_line_vec.push_back(index);
  } else if (event == CV_EVENT_RBUTTONDOWN) {
    SET_PIXEL(image, index, lsd->bck_line_colour);
    bck_line_vec.push_back(index);
  } else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) {
    if (!g_sub_restart && sub_line_vec.size() > 0) {
      Line(image, &sub_line_vec, index, lsd->sub_line_colour);
    }
    SET_PIXEL(image, index, lsd->sub_line_colour);
    sub_line_vec.push_back(index);
    g_sub_restart = false;
    // RemoveLastResult(*lsd->source_image_backup, lsd->source_image);
    // LazySnapping(lsd);
  } else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_RBUTTON)) {
    if (!g_bck_restart && bck_line_vec.size() > 0) {
      Line(image, &bck_line_vec, index, lsd->bck_line_colour);
    }
    SET_PIXEL(image, index, lsd->bck_line_colour);
    bck_line_vec.push_back(index);
    g_bck_restart = false;
  } else if (event == CV_EVENT_LBUTTONUP) {
    g_sub_restart = true;

    RemoveLastResult(*lsd->source_image_backup, lsd->source_image);
    CountTime ct;
    ct.ContBegin();
    LazySnapping(lsd);
    ct.ContEnd();
    ct.ContResult();
  } else if (event == CV_EVENT_RBUTTONUP) {
    g_bck_restart = true;

    RemoveLastResult(*lsd->source_image_backup, lsd->source_image);
    CountTime ct;
    ct.ContBegin();
    LazySnapping(lsd);
    ct.ContEnd();
    ct.ContResult();
  } else {
    is_show = false;
  }
  if (is_show) {
    ShowImage(*image);
  }
}

int main(int argc, char** argv) {
  if (argc == 1) {
    printf("error: need a image\n");
    return 1;
  }
  ImageData<int> src;
  utils::ReadImage(argv[1], &src);
  ImageData<int> src_bk(src);
  ImageData<int> marked_image;

  LazySnappingType lst = WATERSHED;
  LazySnappingData lsd(&src, &src_bk, SUB_COL, BCK_COL, lst);
  namedWindow(WIN_NAME, WINDOW_AUTOSIZE);
  setMouseCallback(WIN_NAME, on_mouse, &lsd);

  ShowImage(src);

  for(;;) {
    int c = waitKey(0);
    switch((char) c) {
    case '\x1b':
      goto exit_main;
    case 'r':
      // reset
      lsd.Reset();
      ShowImage(src);
      lst = WATERSHED;
      lsd.lazy_type = lst;
      break;
    case 'w':
      if (!lsd.marked_image->IsEmpty()) {
        ShowImage(*(lsd.marked_image));
      } else {
        printf("marked image has no data\n");
      }
      break;
    case 's':
      ShowImage(src);
      break;
    case 'p':
      lst = PIXEL;
      lsd.lazy_type = lst;
      break;
    }
  }

exit_main:
    destroyWindow(WIN_NAME);
    utils::SaveImage(IMAGE_OUT_NAME, src_bk);
    return 0;
}
