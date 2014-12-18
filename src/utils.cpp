// Copyright 2014-4 sxniu
#include "include/utils.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <float.h>
#include <opencv/highgui.h>
#include <vector>
#include <stack>
#include <queue>

#include "include/colour.h"
#include "include/ImageData.h"
#include "include/WatershedRegion.h"

#define COMPONENTS 3
#define IN_QUEUE -2
#define CONTOUR_LINE RED

typedef unsigned char uchar;

namespace utils {

void ReadImage(const char* file_name, ImageData<int>* image_data) {
  if (!image_data->IsEmpty()) {
    printf("error: image data must be empty");
    return;
  }
  image_data->m_file_name = file_name;
  int& width = image_data->m_width;
  int& height = image_data->m_height;
  std::vector<int>* data = &(image_data->m_data);

  IplImage* cv_image = cvLoadImage(file_name);

  width = cv_image->width;
  height = cv_image->height;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * cv_image->widthStep + x * COMPONENTS;
      uchar* cv_data = reinterpret_cast<uchar*>(cv_image->imageData);
      int colour = (static_cast<int>(cv_data[index + 2]) << 16) +
                   (static_cast<int>(cv_data[index + 1]) << 8) +
                   (static_cast<int>(cv_data[index]));
      data->push_back(colour);
    }
  }
}

void TurnGray(const ImageData<int>& input_image, ImageData<uchar>* gray_image) {
  if (input_image.IsEmpty()) {
    printf("error: input image data is empty");
    return;
  }
  int height = input_image.GetHeight();
  int width = input_image.GetWidth();

  if (gray_image->IsEmpty()) {
    gray_image->CreateEmptyImage(width, height);
  }
  for (int y = 0; y < height; ++y) {
    for (int x  = 0; x < width; ++x) {
      int index = y * width + x;
      int red = (GET_PIXEL(&input_image, index) & RED) >> 16;
      int green = (GET_PIXEL(&input_image, index) & GREEN) >> 8;
      int blue = GET_PIXEL(&input_image, index) & BLUE;
      uchar gray = static_cast<uchar>(red * 0.3 + green * 0.59 + blue * 0.11);
      SET_PIXEL(gray_image, index, gray);
    }
  }
}

void SaveImage(const char* out_file_name, const ImageData<int>& image_data) {
  if (image_data.IsEmpty()) {
    printf("error: image data is empty\n");
    return;
  }
  int width = image_data.GetWidth();
  int height = image_data.GetHeight();

  CvSize size;
  size.width = width;
  size.height = height;
  IplImage* cv_image = cvCreateImage(size, 8, COMPONENTS);
  if (cv_image == NULL) {
    printf("error: the creation of cv image is failure");
    return;
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * cv_image->widthStep + x * COMPONENTS;
      uchar* cv_data = reinterpret_cast<uchar*>(cv_image->imageData);
      int colour = GET_PIXEL(&image_data, y * width + x);
      cv_data[index + 2] = static_cast<uchar>((colour & RED) >> 16);
      cv_data[index + 1] = static_cast<uchar>((colour & GREEN) >> 8);
      cv_data[index] = static_cast<uchar>(colour & BLUE);
    }
  }
  cvSaveImage(out_file_name, cv_image);
}

void SaveImage(const char* out_file_name, const ImageData<uchar>& image_data) {
  if (image_data.IsEmpty()) {
    printf("error: image data is empty\n");
    return;
  }
  int width = image_data.GetWidth();
  int height = image_data.GetHeight();

  CvSize size;
  size.width = width;
  size.height = height;
  IplImage* cv_image = cvCreateImage(size, 8, COMPONENTS);
  if (cv_image == NULL) {
    printf("error: the creation of cv image is failure");
    return;
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * cv_image->widthStep + x * COMPONENTS;
      uchar* cv_data = reinterpret_cast<uchar*>(cv_image->imageData);
      uchar gray = GET_PIXEL(&image_data, y * width + x);
      cv_data[index + 2] = gray;
      cv_data[index + 1] = gray;
      cv_data[index] = gray;
    }
  }
  cvSaveImage(out_file_name, cv_image);
}

void GetGradiendMap(const ImageData<uchar>& gray_image, ImageData<uchar>* grad_image) {
  int image_width = gray_image.GetWidth();
  int image_height = gray_image.GetHeight();

  if (grad_image != NULL && grad_image->IsEmpty()) {
    grad_image->CreateEmptyImage(image_width, image_height);
  } else {
    printf("error: grad_image should be an empty image\n");
  }

  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      int index_cen = y * image_width + x;
      if (x == 0 || x == image_width - 1 ||
          y == 0 || y == image_height - 1) {
        SET_PIXEL(grad_image, index_cen, 255);
        continue;
      }
      int index[8] = EIGHT_ARROUND_POSITION(x, y, image_width, image_height);

      double gx = GET_PIXEL(&gray_image, index[3]) +
                  2 * GET_PIXEL(&gray_image, index[2]) +
                  GET_PIXEL(&gray_image, index[1]) +
                  -GET_PIXEL(&gray_image, index[5]) +
                  - 2 * GET_PIXEL(&gray_image, index[6]) +
                  - GET_PIXEL(&gray_image, index[7]);

      double gy = GET_PIXEL(&gray_image, index[7]) +
                  2 * GET_PIXEL(&gray_image, index[0]) +
                  GET_PIXEL(&gray_image, index[1]) +
                  - GET_PIXEL(&gray_image, index[5]) +
                  - 2 * GET_PIXEL(&gray_image, index[4]) +
                  - GET_PIXEL(&gray_image, index[3]);

      double sum_of_squares = pow(gx, 2) + pow(gy, 2);
	    uchar dst_gray = std::min(static_cast<int>(sqrt(sum_of_squares)), 255);
      // dst_gray = TURN_COORDINATE_TO_COLOUR(dst_gray, dst_gray, dst_gray);
      SET_PIXEL(grad_image, index_cen, dst_gray);
    }
  }
}

void DoMarkConnectedArea(const ImageData<uchar>& grad_image, ImageData<int>* marked_image,
                         int x, int y, int width, int height, int mark_num,
                         int max_threshold) {

  std::stack<int> unsearched_points;
  unsearched_points.push(y * width + x);
  while (unsearched_points.size() != 0) {
    int index = unsearched_points.top(); 
    unsearched_points.pop();
    int y = index / width;
    int x = index - y * width;
    int arrounds[8] = EIGHT_ARROUND_POSITION(x, y, width, height);
    for (int i = 0; i < 8; ++i) {
      int gradient = static_cast<int>(GET_PIXEL(&grad_image, arrounds[i]));
      int mark_value = GET_PIXEL(marked_image, arrounds[i]);

      assert(mark_value == 0 || std::abs(mark_value) == mark_num);
      if (mark_value == 0) {
        if (gradient <= max_threshold) {
          SET_PIXEL(marked_image, arrounds[i], mark_num);
          unsearched_points.push(arrounds[i]);
        } else {
          SET_PIXEL(marked_image, index, -mark_num);
        }
      }
    }
  }
}

void MarkConnectedArea(const ImageData<uchar>& grad_image,
                       ImageData<int>* marked_image,
                       int max_threshold, int start_mark_num, int* region_count) {
  if (grad_image.IsEmpty()) {
    printf("error: the grad_image is empty\n");
    return;
  }
  int width = grad_image.GetWidth();
  int height = grad_image.GetHeight();
  if (marked_image->IsEmpty()) {
    marked_image->CreateEmptyImage(width, height);
  }

  int mark_num = start_mark_num;
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int index = y * width + x;
      int gradient = static_cast<int>((grad_image.m_data)[index]);
      int mark_value = GET_PIXEL(marked_image, index);
      if (mark_value == 0 && gradient <= max_threshold) {
        SET_PIXEL(marked_image, index, mark_num);
        DoMarkConnectedArea(grad_image, marked_image, x, y,
                            width, height, mark_num,
                            max_threshold);
        mark_num++;
      }
    }
  }
  *region_count = mark_num - start_mark_num;
  // printf("mark_num = %d\n", mark_num);
}

void Watershed(const ImageData<uchar>& grad_image,
               ImageData<int>* marked_image,
               int start_gradient) {
  if (grad_image.IsEmpty()) {
    printf("error: the grad_image is empty\n");
    return;
  }
  int width = grad_image.GetWidth();
  int height = grad_image.GetHeight();
  std::queue<int> grad_queues[256];
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int index = y * width + x;
      int mark_value = GET_PIXEL(marked_image, index);
      if (mark_value < 0 && mark_value != IN_QUEUE) {
        SET_PIXEL(marked_image, index, -mark_value);
        int arrounds[4] = FOUR_ARROUND_POSITION(x, y, width, height);
        for (int i = 0; i < 4; ++i) {
          int arround_mark_value = GET_PIXEL(marked_image, arrounds[i]);
          if (arround_mark_value == 0) {
            int grad_value = (grad_image.m_data)[arrounds[i]];
            assert(grad_value > start_gradient && grad_value < 256);
            grad_queues[grad_value].push(arrounds[i]);
            SET_PIXEL(marked_image, arrounds[i], IN_QUEUE);
          }
        }
      }
    }
  }

  int start_idx = start_gradient;
  for (; start_idx < 256; ++start_idx) {
    if (!grad_queues[start_idx].empty()) {
      break;
    }
  }
  int queues_idx = start_idx;
  while(true) {
    if (grad_queues[queues_idx].empty()) {
      if(++queues_idx >= 256) {
        break;
      }
      continue;
    }
    int mark_index = grad_queues[queues_idx].front();
    grad_queues[queues_idx].pop();
    int mark_value = GET_PIXEL(marked_image, mark_index);
    assert(mark_value == IN_QUEUE);

    int mark_y = mark_index / width;
    int mark_x = mark_index - mark_y * width;
    int mark_arrounds[4] = FOUR_ARROUND_POSITION(mark_x, mark_y, width, height);

    int mark_number = 0;
    for (int i = 0; i < 4; ++i) {
      int mark_value = GET_PIXEL(marked_image, mark_arrounds[i]);
      if (mark_value != WASHED && mark_value != IN_QUEUE && mark_value != 0) {
        if (mark_number == 0) {
          mark_number = mark_value;
          SET_PIXEL(marked_image, mark_index, mark_number);
        } else if (mark_number != mark_value) {
          SET_PIXEL(marked_image, mark_index, WASHED);
          mark_number = WASHED;
        }
      }
    }
    if (mark_number == WASHED) {
      continue;
    }

    for (int i = 0; i < 4; ++i) {
      int mark_value = GET_PIXEL(marked_image, mark_arrounds[i]);
      if (mark_value == 0) {
        int grad_value = static_cast<int>((grad_image.m_data)[mark_arrounds[i]]);
        assert(grad_value > start_gradient);
        grad_queues[grad_value].push(mark_arrounds[i]);
        queues_idx = std::min(queues_idx, grad_value);
        SET_PIXEL(marked_image, mark_arrounds[i], IN_QUEUE);
      }
    }
  }
}

// [start, end)
int GenRanNumI(int seed, int start, int end) {
  srand(seed);
  return (rand() % (end - start)) + start;
}

double GenRanNumD(int seed, int start, int end) {
  srand(seed);
  int integer = (rand() % (end - start)) + start;
  double decimals = rand() / static_cast<double>(RAND_MAX);
  return integer + decimals;
}

// T = uchar or T = int
template <typename T>
void generateCentersPP(const std::vector<T>& image_vec,
                       std::vector<double>* centers, int n,
                       int k, int trials, int random_seed) {
  int N = n;
  int K = k;
  int seed = random_seed;
  (*centers)[0] = GenRanNumI(seed, 0, N);

  int sum = 0;
  std::vector<int> dist1(N);
  std::vector<int> dist2(N);
  std::vector<int> dist3(N);

  const int& center = static_cast<int>(image_vec[(*centers)[0]]);
  for(int i = 0; i < N; i++) {
    const int& data = static_cast<int>(image_vec[i]);
    dist1[i] = COLOUR_DIST(data, center);
    sum += dist1[i];
  }

  for(int cluster = 1; cluster < K; ++cluster) {
    int bestSum = RAND_MAX;
    int bestCenter = -1;

    for(int j = 0; j < trials; j++) {
      double p = GenRanNumD(++seed, 0, 1) * sum;
      int ci = 0;
      for(int i = 0; i < N - 1; i++) {
        if((p -= dist1[i]) <= 0) {
          ci = i;
          break;
        }
      }

      const int& center_ci = static_cast<int>(image_vec[ci]);
      int s = 0;
      for(int i = 0; i < N; i++) {
        const int& data = static_cast<int>(image_vec[i]);
        dist2[i] = std::min(static_cast<int>(COLOUR_DIST(data, center_ci)), dist1[i]);
        s += dist2[i];
      }

      if(s < bestSum) {
        bestSum = s;
        bestCenter = ci;
        std::swap(dist2, dist3);
      }
    }
    (*centers)[cluster] = bestCenter;
    sum = bestSum;
    std::swap(dist1, dist3);
  }

  for(int cluster = 0; cluster < K; ++cluster) {
    (*centers)[cluster] = image_vec[(*centers)[cluster]];
  }
}

void KMeansDistanceComputer(const std::vector<uchar>& gray_vec,
                            std::vector<int>* marked_vec,
                            const std::vector<double>& centers, int k, int n) {
  int K = k;
  int N = n;
  for(int i = 0; i < N; ++i) {
    int sample = static_cast<int>(gray_vec[i]);
    int cluster_best = 0;
    double min_dist = DBL_MAX;

    for(int cluster = 0; cluster < K; ++cluster) {
      const double& center = centers[cluster];
      double dist = (sample - center) * (sample - center);

      if(dist < min_dist) {
        min_dist = dist;
        cluster_best = cluster;
      }
    }
    (*marked_vec)[i] = cluster_best;
  }
}

// centers (K, vector<double>(3))
void KMeansDistanceComputer(const std::vector<int>& colour_vec,
                            std::vector<int>* marked_vec,
                            const std::vector<std::vector<double> >& centers, int k, int n) {
  int K = k;
  int N = n;
  for(int i = 0; i < N; ++i) {
    int sample = colour_vec[i];
    int cluster_best = 0;
    double min_dist = DBL_MAX;

    for(int cluster = 0; cluster < K; ++cluster) {
      int rgb[3] = GET_THREE_COORDINATE(sample);
      double dist = THREE_DIM_DIST(rgb, centers[cluster]);

      if(dist < min_dist) {
        min_dist = dist;
        cluster_best = cluster;
      }
    }
    (*marked_vec)[i] = cluster_best;
  }
}

void Kmeans(const std::vector<uchar>& gray_vec,
            std::vector<int>* marked_vec,
            std::vector<double>* k_mean_set,
            int k, int iter, int random_seed) {
  int N = gray_vec.size();
  int K = k;
  int times = K != 1 ? iter : 2;
  if (marked_vec->size() == 0) {
    printf("error: the marked_vec need N size\n");
    return;
  }

  std::vector<double> centers(K);
  std::vector<double> old_centers(K);
  for (int j = 0; j < times; ++j) {
    std::vector<int> counters(K, 0);
    double max_center_shift = DBL_MAX;
    if (j == 0) {
      // Arthur & Vassilvitskii (2007) k-means++: The Advantages of Careful Seeding
      int trials = 3;
      generateCentersPP(gray_vec, &centers, N, K, trials, random_seed);
    } else {
      max_center_shift = 0;
      for(int i = 0; i < N; ++i) {
        int sample = static_cast<int>(gray_vec[i]);
        int k_value = (*marked_vec)[i];
        assert(k_value >= 0 && k_value < K);
        centers[k_value] += sample;
        counters[k_value]++;
      }

      for(int cluster = 0; cluster < K; cluster++) {
        if(counters[cluster] != 0)
            continue;

        // if some cluster appeared to be empty then:
        //   1. find the biggest cluster
        //   2. find the farthest from the center point in the biggest cluster
        //   3. exclude the farthest point from the biggest cluster and form a new 1-point cluster.
        int max_cluster = 0;
        for(int i = 1; i < K; ++i) {
          if(counters[max_cluster] < counters[i]) {
            max_cluster = i;
          }
        }

        double max_dist = 0;
        int farthest_i = -1;
        double mean_max_cluster = centers[max_cluster] / counters[max_cluster];

        for(int i = 0; i < N; i++) {
          int k = (*marked_vec)[i];
          if(k != max_cluster)
            continue;
          int sample = static_cast<int>(gray_vec[i]);
          double dist = (sample - mean_max_cluster) * (sample - mean_max_cluster);

          if(max_dist <= dist) {
            max_dist = dist;
            farthest_i = i;
          }
        }

        counters[max_cluster]--;
        counters[cluster]++;
        (*marked_vec)[farthest_i] = cluster;

        int data = static_cast<int>(gray_vec[farthest_i]);
        centers[max_cluster] -= data;
        centers[cluster] += data;
      }

      for(int cluster = 0; cluster < K; ++cluster) {
         centers[cluster] /= counters[cluster];
         if(j > 0) {
           double t = centers[cluster] - old_centers[cluster];
           double dist = t * t;
           max_center_shift = std::max(max_center_shift, dist);
         }
      }
    }

    if (max_center_shift <= EPSILON) {
      // get mean value of clusters
      if (k_mean_set != NULL) {
        for (int i = 0; i < K; ++i) {
          (*k_mean_set)[i] = centers[i];
        }
      }
      break;
    }

    KMeansDistanceComputer(gray_vec, marked_vec, centers, K, N);
    std::swap(centers, old_centers);
    for (int cluster = 0; cluster < K; ++cluster) {
      centers[cluster] = 0;
    }
  }

  // get mean value of clusters
  if (k_mean_set != NULL && (*k_mean_set)[0] == 0) {
    for (int i = 0; i < K; ++i) {
      (*k_mean_set)[i] = old_centers[i];
    }
  }
}

void Kmeans(const std::vector<int>& colour_vec,
            std::vector<int>* marked_vec,
            std::vector<std::vector<double> >* k_mean_colour_set,
            int k, int iter, int random_seed) {
  if (marked_vec == NULL) {
    printf("error: the marked_vec is null\n");
    return;
  }
  int N = colour_vec.size();
  int K = k;
  int times = K != 1 ? iter : 2;
  if (marked_vec->size() == 0) {
    printf("error: the marked_vec need N size\n");
    return;
  }

  std::vector<std::vector<double> > centers(K, std::vector<double>(3, 0));
  std::vector<std::vector<double> > old_centers(K, std::vector<double>(3, 0));
  for (int j = 0; j < times; ++j) {
    std::vector<int> counters(K, 0);
    double max_center_shift = DBL_MAX;
    if (j == 0) {
      // Arthur & Vassilvitskii (2007) k-means++: The Advantages of Careful Seeding
      int trials = 3;
      std::vector<double> center_zero(K);
      generateCentersPP(colour_vec, &center_zero, N, K, trials, random_seed);
      for (int i = 0; i < K; ++i) {
        int colour = static_cast<int>(center_zero[i]);
        int rgb[3] = GET_THREE_COORDINATE(colour);
        centers[i][0] = rgb[0];
        centers[i][1] = rgb[1];
        centers[i][2] = rgb[2];
      }
    } else {
      max_center_shift = 0;
      for(int i = 0; i < N; ++i) {
        int k_value = (*marked_vec)[i];
        assert(k_value >= 0 && k_value < K);
        int rgb[3] = GET_THREE_COORDINATE(colour_vec[i]);
        centers[k_value][0] += rgb[0];
        centers[k_value][1] += rgb[1];
        centers[k_value][2] += rgb[2];
        counters[k_value]++;
      }

      for(int cluster = 0; cluster < K; cluster++) {
        if(counters[cluster] != 0)
            continue;

        // if some cluster appeared to be empty then:
        //   1. find the biggest cluster
        //   2. find the farthest from the center point in the biggest cluster
        //   3. exclude the farthest point from the biggest cluster and form a new 1-point cluster.
        int max_cluster = 0;
        for(int i = 1; i < K; ++i) {
          if(counters[max_cluster] < counters[i]) {
            max_cluster = i;
          }
        }

        double max_dist = 0;
        int farthest_i = -1;
        std::vector<double> mean_max_cluster(3);
        mean_max_cluster[0] = centers[max_cluster][0] / counters[max_cluster];
        mean_max_cluster[1] = centers[max_cluster][1] / counters[max_cluster];
        mean_max_cluster[2] = centers[max_cluster][2] / counters[max_cluster];

        for(int i = 0; i < N; i++) {
          int k = (*marked_vec)[i];
          if(k != max_cluster)
            continue;
          int rgb[3] = GET_THREE_COORDINATE(colour_vec[i]);
          double dist = THREE_DIM_DIST(rgb, mean_max_cluster);

          if(max_dist <= dist) {
            max_dist = dist;
            farthest_i = i;
          }
        }

        counters[max_cluster]--;
        counters[cluster]++;
        (*marked_vec)[farthest_i] = cluster;

        int rgb[3] = GET_THREE_COORDINATE(colour_vec[farthest_i]);
        centers[max_cluster][0] -= rgb[0];
        centers[max_cluster][1] -= rgb[1];
        centers[max_cluster][2] -= rgb[2];
        centers[cluster][0] += rgb[0];
        centers[cluster][1] += rgb[1];
        centers[cluster][2] += rgb[2];
      }

      for(int cluster = 0; cluster < K; ++cluster) {
        centers[cluster][0] /= counters[cluster];
        centers[cluster][1] /= counters[cluster];
        centers[cluster][2] /= counters[cluster];
        if(j > 0) {
          double dist = THREE_DIM_DIST(centers[cluster], old_centers[cluster]);
          max_center_shift = std::max(max_center_shift, dist);
        }
      }
    }

    if (max_center_shift <= EPSILON) {
      // get mean value of clusters
      if (k_mean_colour_set != NULL) {
        for (int i = 0; i < K; ++i) {
          (*k_mean_colour_set)[i][0] = centers[i][0];
          (*k_mean_colour_set)[i][1] = centers[i][1];
          (*k_mean_colour_set)[i][2] = centers[i][2];
        }
      }
      break;
    }

    KMeansDistanceComputer(colour_vec, marked_vec, centers, K, N);
    std::swap(centers, old_centers);
    for (int cluster = 0; cluster < K; ++cluster) {
      centers[cluster][0] = 0;
      centers[cluster][1] = 0;
      centers[cluster][2] = 0;
    }
  }

  // get mean value of clusters
  if (k_mean_colour_set != NULL && (*k_mean_colour_set)[0][0] == 0) {
    for (int i = 0; i < K; ++i) {
      (*k_mean_colour_set)[i][0] = old_centers[i][0];
      (*k_mean_colour_set)[i][1] = old_centers[i][1];
      (*k_mean_colour_set)[i][2] = old_centers[i][2];
    }
  }
}

void ShowMarkedImage(ImageData<int>* marked_image) {
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int data = GET_PIXEL(marked_image, index);
      SET_PIXEL(marked_image, index, data * 10000 + 100);
    }
  }
}

/*
  Calculate beta - parameter of GrabCut algorithm.
  beta = 1/(2*avg(sqr(||color[i] - color[j]||)))
*/
double CalcBeta(const ImageData<int>& img) {
  double beta = 0;
  int width = img.GetWidth();
  int height = img.GetHeight();
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      int color = GET_PIXEL(&img, y * width + x);
      // left
      if(x > 0) {
        int color_arr = GET_PIXEL(&img, y * width + x - 1);
        beta += COLOUR_DIST_SQUARE(color, color_arr);
      }
      // upleft
      if(y > 0 && x > 0) {
        int color_arr = GET_PIXEL(&img, (y - 1) * width + x - 1);
        beta += COLOUR_DIST_SQUARE(color, color_arr);
      }
      // up
      if(y > 0) {
        int color_arr = GET_PIXEL(&img, (y - 1) * width + x);
        beta += COLOUR_DIST_SQUARE(color, color_arr);
      }
      // upright
      if(y > 0 && x < width - 1) {
        int color_arr = GET_PIXEL(&img, (y - 1) * width + x + 1);
        beta += COLOUR_DIST_SQUARE(color, color_arr);
      }
    }
  }
  if(beta <= EPSILON) {
    beta = 0;
  } else {
    beta = 1.f / (2 * beta / (4 * width * height - 3 * width - 3 * height + 2));
  }
  return beta;
}

#if 0
void GraphCut(const ImageData<int>& source_image,
              ImageData<int>* marked_image,
              Segmentation* seg,
              int undef_colour,
              int sub_colour) {
  int width = source_image.GetWidth();
  int height = source_image.GetHeight();
  if (marked_image->IsEmpty()) {
    marked_image->CreateEmptyImage(width, height);
  }
  int vtx_count = width * height;
  int edge_count = 2 * (4 * vtx_count - 3 * (width + height) + 2);
  GraphType graph(vtx_count, edge_count);

  // e1[0] is background, e1[1] is subject
  double e1[2] = {0, 0};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&source_image, index);
      int marked_colour = GET_PIXEL(marked_image, index);
      if (marked_colour == undef_colour) {
        e1[0] = seg->GetEnergyRegionItem(colour, BACKGROUND);
        e1[1] = seg->GetEnergyRegionItem(colour, SUBJECT);
      } else if (marked_colour == sub_colour) {
        e1[0] = DBL_MAX;
        e1[1] = 0;
      } else {
        e1[0] = 0;
        e1[1] = DBL_MAX;
      }
      graph.add_node();
      graph.add_tweights(index, e1[0], e1[1]);

      // 8 neighbours
      if (x > 0) {
        int near_colour = GET_PIXEL(&source_image, index - 1);
        double e2 = seg->GetEnergyBoundaryItem(colour, near_colour, LEFT);
        graph.add_edge(index, index - 1, e2, e2);
      }
      if (x > 0 && y > 0) {
        int near_colour = GET_PIXEL(&source_image, index - width - 1);
        double e2 = seg->GetEnergyBoundaryItem(colour, near_colour, LEFT_UP);
        graph.add_edge(index, index - width - 1, e2, e2);
      }
      if (y > 0) {
        int near_colour = GET_PIXEL(&source_image, index - near_colour);
        double e2 = seg->GetEnergyBoundaryItem(colour, near_colour, UP);
        graph.add_edge(index, index - width, e2, e2);
      }
      if (x < width - 1 && y > 0) {
        int near_colour = GET_PIXEL(&source_image, index - width + 1);
        double e2 = seg->GetEnergyBoundaryItem(colour, near_colour, RIGHT_UP);
        graph.add_edge(index, index - width + 1, e2, e2);
      }
    }
  }

  int flow = graph.maxflow();

  seg->SegmentImageByGraph(graph, marked_image);
}
#endif

void ExtractContourLine(const WatershedRegionGroup& wrg, ImageData<int>* source_image,
                        ImageData<int>* marked_image) {
  if (source_image->IsEmpty() || marked_image->IsEmpty()) {
    printf("error: source_image or marked_image is empty");
    return;
  }
  int width = source_image->GetWidth();
  int height = source_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_value = GET_PIXEL(marked_image, index);
      if (marked_value > 0) {
        int colour = GET_PIXEL(source_image, index);
        SET_PIXEL(marked_image, index, colour);
      }
    }
  }
  int region_count = wrg.GetRegionCount();
  for (int i = 0; i < region_count; ++i) {
    const WatershedRegionInfo& wri = GET_REGION_ITEM(wrg, i);
    if (wri.m_scene == SUBJECT) {
      for (int j = 0; j < wri.m_adjacent_regions.size(); ++j) {
        WatershedRegionInfo* adj_wri = wri.m_adjacent_regions[j];
        if (adj_wri->m_scene == BACKGROUND) {
          int adj_num = adj_wri->m_region_num;
          const std::vector<int>& watershed_vec =
            wri.m_watershed_points.find(adj_num)->second;
          for (int k = 0; k < watershed_vec.size(); ++k) {
            SET_PIXEL(source_image, watershed_vec[k], CONTOUR_LINE);
            SET_PIXEL(marked_image, watershed_vec[k], RED);
          }
        }
      }
    } else {
      for (int j = 0; j < wri.m_adjacent_regions.size(); ++j) {
        WatershedRegionInfo* adj_wri = wri.m_adjacent_regions[j];
        if (adj_wri->m_scene == SUBJECT) {
          int adj_num = adj_wri->m_region_num;
          const std::vector<int>& watershed_vec =
            wri.m_watershed_points.find(adj_num)->second;
          for (int k = 0; k < watershed_vec.size(); ++k) {
            SET_PIXEL(source_image, watershed_vec[k], CONTOUR_LINE);
            SET_PIXEL(marked_image, watershed_vec[k], RED);
          }
        }
      }
    }
  }
}

void ExtractContourLine(ImageData<int>* source_image,
                        ImageData<int>* marked_image) {
  if (source_image->IsEmpty() || marked_image->IsEmpty()) {
    printf("error: source_image or marked_image is empty");
    return;
  }
  int width = source_image->GetWidth();
  int height = source_image->GetHeight();
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      int index = y * width + x;
      int arrounds[8] = EIGHT_ARROUND_POSITION(x, y, width, height);
      int cen_colour = GET_PIXEL(marked_image, index);
      if (cen_colour == IGNORED) {
        continue;
      }
      for (int i = 0; i < 8; ++i) {
        int arr_colour = GET_PIXEL(marked_image, arrounds[i]);
        if (cen_colour != arr_colour && arr_colour == SUBJECT) {
          SET_PIXEL(source_image, index, CONTOUR_LINE);
          break;
        }
      }
    }
  }
}

void CreateSegImage(const ImageData<int>& marked_image,
                    const WatershedRegionGroup& wrg,
                    ImageData<int>* seg_image) {
  int width = marked_image.GetWidth();
  int height = marked_image.GetHeight();
  if (seg_image->IsEmpty()) {
    seg_image->CreateEmptyImage(width, height);
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int mark_num = GET_PIXEL(&marked_image, index);
      if (mark_num <= 0) {
        continue;
      }
      int region_index = mark_num - wrg.m_region_num_offset;
      const WatershedRegionInfo& wri = GET_REGION_ITEM(wrg, region_index);
      if (wri.m_scene == SUBJECT) {
        SET_PIXEL(seg_image, index, WHITE);
      } else {
        SET_PIXEL(seg_image, index, BLACK);
      }
    }
  }
}

void ExtractMarkPoints(const ImageData<int>& mask_image,
                       const ImageData<int>& source_image,
                       int mark_colour,
                       std::vector<int>* mark_points,
                       std::vector<int>* mark_index) {
  int width = mask_image.GetWidth();
  int height = mask_image.GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&mask_image, index);
      if (colour == mark_colour) {
        int value = GET_PIXEL(&source_image, index);
        mark_points->push_back(value);
        mark_index->push_back(index);
      }
    }
  }
}

// scale_factor is the factor of both width and height of the image
void Scale(const ImageData<int>& src_image, ImageData<int>* dst_image, double scale_factor) {
  assert(dst_image != NULL);
  int src_width = src_image.GetWidth();
  int src_height = src_image.GetHeight();
  int dst_width = static_cast<int>(src_width * scale_factor);
  int dst_height = static_cast<int>(src_height * scale_factor);

  if ((scale_factor >= 2 || (scale_factor <= 0.5 && scale_factor > 0)) &&
      dst_image->IsEmpty()) {
    dst_image->CreateEmptyImage(dst_width, dst_height);
  } else if (dst_image->IsEmpty() == false) {
    assert(dst_width == dst_image->GetWidth() && dst_height == dst_image->GetHeight());
  } else {
    printf("error: the dst_image must be empty or the scale_factor is not supported\n");
    return;
  }
  // up scale
  if (scale_factor >= 2) {
    for (int y = 0; y < src_height; ++y) {
      for (int x = 0; x < src_width; ++x) {
        int colour = GET_PIXEL(&src_image, y * src_width + x);
        int new_y = static_cast<int>(y * scale_factor);
        int new_x = static_cast<int>(x * scale_factor);
        SET_PIXEL(dst_image, new_y * dst_width + new_x, colour);
        SET_PIXEL(dst_image, new_y * dst_width + new_x + 1, colour);
        SET_PIXEL(dst_image, (new_y + 1) * dst_width + new_x, colour);
        SET_PIXEL(dst_image, (new_y + 1) * dst_width + new_x + 1, colour);
      }
    }
    return;
  }

  // down scale
  const int inv_factor = static_cast<int>(1 / scale_factor);
  int k = 0;
  for (int y = 0; y < src_height; ++y) {
    if (y % inv_factor != 0) {
      continue;
    }
    for (int x = 0; x < src_width; ++x) {
      if (x % inv_factor == 0) {
        int index = y * src_width + x;
        int colour = GET_PIXEL(&src_image, index);
        SET_PIXEL(dst_image, k++, colour);
      }
    }
  }
}

void HalfScale(const ImageData<int>& src_image, ImageData<int>* dst_image) {
  Scale(src_image, dst_image, 0.5);
}

void DoubleScale(const ImageData<int>& src_image, ImageData<int>* dst_image) {
  utils::Scale(src_image, dst_image, 2);
}

}  // namespace utils
