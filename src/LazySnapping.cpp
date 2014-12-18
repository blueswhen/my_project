// Copyright sxniu 2014-9
#include "include/LazySnapping.h"

#include <vector>
#include <map>
#include <stdio.h>
#include <float.h>
#include <assert.h>
#include <opencv/highgui.h>

#include "include/ImageData.h"
#include "include/utils.h"
#include "include/colour.h"
#include "include/CountTime.h"
#include "include/WatershedRegion.h"
#include "include/ui.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/Graph.h"
#include "include/gcgraph.hpp"
#include "include/maxflow-v3.03/graph.h"

#define START_GRADIENT 10
// START_MARK_NUM must be positive number
#define START_MARK_NUM 100
#define K_NUM 60
#define ITER 10
#define LAMDA 60
typedef Graph<double, double, double> GraphType;

using namespace cv;

inline double GetColourDistence(double c_mean[3],
                         const std::vector<std::vector<double> >& k_means) {
  double min = DBL_MAX;
  int n = k_means.size();
  for (int i = 0; i < n; ++i) {
    double diff = THREE_DIM_DIST_SQUARE(c_mean, k_means[i]);
    if (diff < min) {
      min = diff;
    }
  }
  return min;
}

inline double GetColourDistence(const int& colour,
                         const std::vector<std::vector<double> >& k_means) {
  double rgb[3] = GET_THREE_COORDINATE(colour);
  return GetColourDistence(rgb, k_means);
}

void GraphCutBaseWatershed(const std::vector<std::vector<double> >& k_means_sub,
                           const std::vector<std::vector<double> >& k_means_bck,
                           WatershedRegionGroup* wrg) {
  int region_count = wrg->GetRegionCount();
  GraphType graph(region_count, region_count * 4);
  for (int i = 0; i < region_count; ++i) {
    WatershedRegionInfo& wri = GET_REGION_ITEM(*wrg, i);
    // region_mean has three colour channels
    const std::vector<double>& region_mean = wri.GetRegionMeanValue();
    double rgb[3] = {region_mean[0], region_mean[1], region_mean[2]};
    double sub_dist = GetColourDistence(rgb, k_means_sub);
    double bck_dist = GetColourDistence(rgb, k_means_bck);
    double sum_dist = sub_dist + bck_dist;

    // e1[0] is background, e1[1] is subject
    double e1[2];
    if (wri.m_scene == UNDEFINE) {
      e1[0] = bck_dist / sum_dist;
      e1[1] = sub_dist / sum_dist;
    } else if (wri.m_scene == SUBJECT) {
      e1[0] = DBL_MAX;
      e1[1] = 0;
    } else {
      e1[0] = 0;
      e1[1] = DBL_MAX;
    }
    graph.add_node();
    graph.add_tweights(i, e1[0], e1[1]);

    for (int j = 0; j < wri.m_adjacent_regions.size(); ++j) {
      WatershedRegionInfo* adj_wri = wri.m_adjacent_regions[j];
      const std::vector<double>& adj_region_mean = adj_wri->GetRegionMeanValue();
      double e2 = 0;
      e2 = LAMDA / (THREE_DIM_DIST(region_mean, adj_region_mean) + 1);

      int adj_region_num = adj_wri->m_region_num;
      int adj_region_index = adj_region_num - wrg->m_region_num_offset;
      assert(adj_region_index < i);
      graph.add_edge(i, adj_region_index, e2, e2);
    }
  }

  graph.maxflow();

  for (int i = 0; i < region_count; ++i) {
    WatershedRegionInfo& wri = GET_REGION_ITEM(*wrg, i);
    if (wri.m_scene == UNDEFINE) {
      if (graph.what_segment(i) == GraphType::SOURCE) {
        wri.m_scene = SUBJECT;
      } else {
        wri.m_scene = BACKGROUND;
      }
    }
  }
}

void GraphCutBasePixel(const std::vector<std::vector<double> >& k_means_sub,
                       const std::vector<std::vector<double> >& k_means_bck,
                       const ImageData<int>& source_image,
                       ImageData<int>* marked_image,
                       std::map<int, int>* graph_vtx_map) {
#define BUILD_VTX(index) \
({ \
  graph_vtx_map != NULL ? (*graph_vtx_map)[index] : index; \
})

#define IS_BUILD_EDGE(index) \
  (graph_vtx_map != NULL && graph_vtx_map->find(index) != graph_vtx_map->end())

  int width = source_image.GetWidth();
  int height = source_image.GetHeight();
  if (marked_image->IsEmpty()) {
    marked_image->CreateEmptyImage(width, height);
  }
  int vtx_count = width * height;
  int edge_count = 2 * (4 * vtx_count - 3 * (width + height) + 2);

#define MY_MAXFLOW 1
#define B_MAXFLOW 0
#define OPENCV_MAXFLOW 0

#if MY_MAXFLOW
  user::Graph<double> graph(vtx_count, edge_count);
#elif B_MAXFLOW
  GraphType graph(vtx_count, edge_count);
#elif OPENCV_MAXFLOW
  GCGraph<double> graph;
  graph.create(vtx_count, edge_count);
#endif

  // e1[0] is background, e1[1] is subject
  double e1[2] = {0, 0};
  const double lamda = LAMDA;
  const double lamda_div_sqrt2 = LAMDA / std::sqrt(2.0f);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_colour = GET_PIXEL(marked_image, index);
      if (marked_colour == IGNORED) {
        continue;
      }

      int colour = GET_PIXEL(&source_image, index);
      // two GetColourDistence functions spend 75% time in whole cycle
      double sub_dist = GetColourDistence(colour, k_means_sub);
      double bck_dist = GetColourDistence(colour, k_means_bck);

      double sum_dist = sub_dist + bck_dist;
      if (marked_colour == UNDEFINE) {
        e1[0] = bck_dist / sum_dist;
        e1[1] = sub_dist / sum_dist;
      } else if (marked_colour == SUBJECT) {
        e1[0] = DBL_MAX;
        e1[1] = 0;
      } else {
        e1[0] = 0;
        e1[1] = DBL_MAX;
      }
      int vtx0 = BUILD_VTX(index);
#if MY_MAXFLOW
      graph.AddNode(vtx0, e1[0], e1[1]);
#elif B_MAXFLOW
      graph.add_node();
      graph.add_tweights(vtx0, e1[0], e1[1]);
#elif OPENCV_MAXFLOW
      graph.addVtx();
      graph.addTermWeights(index, e1[0], e1[1]);
#endif

#if 1
      // 8 neighbours
      if ((graph_vtx_map == NULL && x > 0) || IS_BUILD_EDGE(index - 1)) {
        int near_colour = GET_PIXEL(&source_image, index - 1);
        double e2 = lamda / (COLOUR_DIST(colour, near_colour) + 0.01);
        int vtx1 = BUILD_VTX(index - 1);
#if MY_MAXFLOW
        graph.AddEdge(vtx0, vtx1, e2);
#elif B_MAXFLOW
        graph.add_edge(vtx0, vtx1, e2, e2);
#elif OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#endif
      }
      if ((graph_vtx_map == NULL && x > 0 && y > 0) || IS_BUILD_EDGE(index - width - 1)) {
        int near_colour = GET_PIXEL(&source_image, index - width - 1);
        double e2 = lamda_div_sqrt2 / (COLOUR_DIST(colour, near_colour) + 0.01);
        int vtx1 = BUILD_VTX(index - width - 1);
#if MY_MAXFLOW
        graph.AddEdge(vtx0, vtx1, e2);
#elif B_MAXFLOW
        graph.add_edge(vtx0, vtx1, e2, e2);
#elif OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#endif
      }
      if ((graph_vtx_map == NULL && y > 0) || IS_BUILD_EDGE(index - width)) {
        int near_colour = GET_PIXEL(&source_image, index - width);
        double e2 = lamda / (COLOUR_DIST(colour, near_colour) + 0.01);
        int vtx1 = BUILD_VTX(index - width);
#if MY_MAXFLOW
        graph.AddEdge(vtx0, vtx1, e2);
#elif B_MAXFLOW
        graph.add_edge(vtx0, vtx1, e2, e2);
#elif OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#endif
      }
      if ((graph_vtx_map == NULL && x < width - 1 && y > 0) || IS_BUILD_EDGE(index - width + 1)) {
        int near_colour = GET_PIXEL(&source_image, index - width + 1);
        double e2 = lamda_div_sqrt2 / (COLOUR_DIST(colour, near_colour) + 0.01);
        int vtx1 = BUILD_VTX(index - width + 1);
#if MY_MAXFLOW
        graph.AddEdge(vtx0, vtx1, e2);
#elif B_MAXFLOW
        graph.add_edge(vtx0, vtx1, e2, e2);
#elif OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#endif
      }
#endif
    }
  }

#if MY_MAXFLOW
  graph.MaxFlow();
#elif B_MAXFLOW
  graph.maxflow();
#elif OPENCV_MAXFLOW
  graph.maxFlow();
#endif

#if 1
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_colour = GET_PIXEL(marked_image, index);
      if (marked_colour != IGNORED) {
        int vtx0 = BUILD_VTX(index);
#if MY_MAXFLOW
        if (graph.IsBelongToSource(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#elif B_MAXFLOW
        if (graph.what_segment(vtx0) == GraphType::SOURCE) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#elif OPENCV_MAXFLOW
        if (graph.inSourceSegment(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#endif
      }
    }
  }
#endif
}

LazySnapping::LazySnapping(SegmentationData* sd, UserInput* usr_input)
  : Segmentation(sd, usr_input)
  , m_lazy_type(PIXEL) {}

void LazySnapping::InitMarkedImage(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  std::vector<int>* sub_mark_index = uip->GetSubjectPoints().first;
  std::vector<int>* bck_mark_index = uip->GetBackgroundPoints().first;
  assert(sub_mark_index != NULL && bck_mark_index != NULL);
  ImageData<int>* marked_image = sd->GetMarkedImage();
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      SET_PIXEL(marked_image, y * width + x, UNDEFINE);
    }
  }
  for (int i = 0; i < sub_mark_index->size(); ++i) {
    SET_PIXEL(marked_image, (*sub_mark_index)[i], SUBJECT);
  }
  for (int i = 0; i < bck_mark_index->size(); ++i) {
    SET_PIXEL(marked_image, (*bck_mark_index)[i], BACKGROUND);
  }
}

void LazySnapping::Cut(SegmentationData* sd, UserInput* uip, std::map<int, int>* graph_vtx_map) {
  assert(sd != NULL && uip != NULL);
  Segmentation::RemoveLastResult(sd);
  // get user input data, these code needn't to change
  // whatever the situation is lines or square or lasso
  ImageData<int>* ui_image = sd->GetSourceImage();
  const ImageData<int>* source_image = sd->GetSourceImageBck();
  int subject_colour = sd->GetSubjectColour();
  int background_colour = sd->GetBackgroundColour();
  ImageData<int>* marked_image = sd->GetMarkedImage();
  std::vector<int>* sub_mark_index = uip->GetSubjectPoints().first;
  std::vector<int>* sub_mark_value = uip->GetSubjectPoints().second;
  std::vector<int>* bck_mark_index = uip->GetBackgroundPoints().first;
  std::vector<int>* bck_mark_value = uip->GetBackgroundPoints().second;

  std::vector<std::vector<double> > k_means_sub(K_NUM, std::vector<double>(3));
  std::vector<std::vector<double> > k_means_bck(K_NUM, std::vector<double>(3));
  std::vector<int> cluster_vec_sub(sub_mark_value->size());
  std::vector<int> cluster_vec_bck(bck_mark_value->size());
  utils::Kmeans(*sub_mark_value, &cluster_vec_sub, &k_means_sub,
                K_NUM, ITER, ui_image->GetRandomSeed());
  utils::Kmeans(*bck_mark_value, &cluster_vec_bck, &k_means_bck,
                K_NUM, ITER, ui_image->GetRandomSeed());

  if (m_lazy_type == LazySnapping::WATERSHED) {
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
                             *sub_mark_index, *bck_mark_index,
                             region_count, START_MARK_NUM);
    GraphCutBaseWatershed(k_means_sub, k_means_bck, &wrg);
    utils::ExtractContourLine(wrg, ui_image, marked_image);
  } else {
    GraphCutBasePixel(k_means_sub, k_means_bck, *source_image, marked_image, graph_vtx_map);
    utils::ExtractContourLine(ui_image, marked_image);
  }
  sd->SetCutStatus(true);
}

void LazySnapping::MakeTrimapForMarkedImage(ImageData<int>* marked_image, int band_width) {
  assert(marked_image != NULL);
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  // define segment line
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int cen = y * width + x;
      int arr[8] = EIGHT_ARROUND_POSITION(x, y, width, height);
      int cen_colour = GET_PIXEL(marked_image, cen);
      if (cen_colour == SUBJECT) {
        for (int i = 0; i < 8; ++i) {
          int arr_colour = GET_PIXEL(marked_image, arr[i]);
          if (arr_colour == BACKGROUND) {
            SET_PIXEL(marked_image, cen, TEMP1);
            break;
          }
        }
      }
    }
  }
  // dilate
  std::vector<int> sub_vec;
  std::vector<int> bck_vec;
  std::vector<int> undef_vec;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(marked_image, index);
      if (colour ==  TEMP1) {
        undef_vec.push_back(index);
        // SET_PIXEL(marked_image, index, UNDEFINE);
        int scene_colour[4] = {TEMP2, TEMP2, TEMP2, TEMP2};
        for (int delt = 1; delt <= band_width; ++delt) {
          int idx[4] = {y * width + max(x - delt, 0),
                        y * width + min(x + delt, width - 1),
                        max(y - delt, 0) * width + x, 
                        min(y + delt, height - 1) * width + x};
          int col[4] = {TEMP2, TEMP2, TEMP2, TEMP2};
          for (int i = 0; i < 4; ++i) {
            col[i] = GET_PIXEL(marked_image, idx[i]);
          }
          for (int i = 0; i < 4; ++i) {
            if (col[i] != TEMP1) {
              if (col[i] == SUBJECT || col[i] == BACKGROUND) {
                scene_colour[i] = col[i];
              }
              if (scene_colour[i] == SUBJECT) {
                sub_vec.push_back(idx[i]);
              } else if (scene_colour[i] == BACKGROUND) {
                bck_vec.push_back(idx[i]);
              }
              undef_vec.push_back(idx[i]);
              // SET_PIXEL(marked_image, idx[i], UNDEFINE);
            }
          }
        }
      }
    }
  }
  // set ignored colour
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int cen = y * width + x;
      int cen_colour = GET_PIXEL(marked_image, cen);
      if (cen_colour != IGNORED) {
        SET_PIXEL(marked_image, cen, IGNORED);
      }
    }
  }
  for (int i = 0; i < undef_vec.size(); ++i) {
    SET_PIXEL(marked_image, undef_vec[i], UNDEFINE);
  }
  // add sub line and bck line
  for (int i = 0; i < sub_vec.size(); ++i) {
    int y = sub_vec[i] / width;
    int x = sub_vec[i] - y * width;
    int arr[4] = FOUR_ARROUND_POSITION(x, y, width, height);
    for (int k = 0; k < 4; ++k) {
      int arr_colour = GET_PIXEL(marked_image, arr[k]);
      if (arr_colour == IGNORED) {
        SET_PIXEL(marked_image, sub_vec[i], SUBJECT);
      }
    }
  }
  for (int i = 0; i < bck_vec.size(); ++i) {
    int y = bck_vec[i] / width;
    int x = bck_vec[i] - y * width;
    int arr[4] = FOUR_ARROUND_POSITION(x, y, width, height);
    for (int k = 0; k < 4; ++k) {
      int arr_colour = GET_PIXEL(marked_image, arr[k]);
      if (arr_colour == IGNORED) {
        SET_PIXEL(marked_image, bck_vec[i], BACKGROUND);
      }
    }
  }
}

void LazySnapping::MakeGraphVtx(const ImageData<int>& marked_image, std::map<int, int>* graph_vtx_map) {
  int width = marked_image.GetWidth();
  int height = marked_image.GetHeight();
  int i = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&marked_image, index);
      if (colour != IGNORED) {
        (*graph_vtx_map)[index] = i++;
      }
    }
  }
}

void LazySnapping::UpdateSceneVector(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  uip->Reset();
  ImageData<int>* marked_image = sd->GetMarkedImage();
  ImageData<int>* src_image = sd->GetSourceImageBck();
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  std::vector<int>* sub_mark_index = uip->GetSubjectPoints().first;
  std::vector<int>* sub_mark_value = uip->GetSubjectPoints().second;
  std::vector<int>* bck_mark_index = uip->GetBackgroundPoints().first;
  std::vector<int>* bck_mark_value = uip->GetBackgroundPoints().second;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_colour = GET_PIXEL(marked_image, index);
      int src_colour = GET_PIXEL(src_image, index);
      if (marked_colour == SUBJECT) {
        sub_mark_index->push_back(index);
        sub_mark_value->push_back(src_colour);
      } else if (marked_colour == BACKGROUND) {
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(src_colour);
      }
    }
  }
}

void LazySnapping::DoPartition() {
  CountTime ct;
  ct.ContBegin();

  SegmentationWithoutCoarsen();
  // SegmentationWithCoarsen();

  ct.ContEnd();
  ct.PrintTime();
}

ImageData<int>* LazySnapping::GetUiImage() {
  return m_sd->GetSourceImage();
}

void LazySnapping::DoLeftButtonDown(int x, int y) {
  m_usr_input->DrawFirstPointForSub(x, y);
}

void LazySnapping::DoRightButtonDown(int x, int y) {
  m_usr_input->DrawFirstPointForBck(x, y);
}

void LazySnapping::DoLeftMouseMove(int x, int y) {
  m_usr_input->DrawSubjectBegin(x, y);
}

void LazySnapping::DoRightMouseMove(int x, int y) {
  m_usr_input->DrawBackgroundBegin(x, y);
}

void LazySnapping::DoLeftButtonUp(int x, int y) {
  m_usr_input->DrawSubjectFinish(x, y);
  DoPartition();
}

void LazySnapping::DoRightButtonUp(int x, int y) {
  m_usr_input->DrawBackgroundFinish(x, y);
  DoPartition();
}

void LazySnapping::ResetUserInput() {
  Segmentation::ResetUserInput();
  SetLazySnappingMethod(LazySnapping::PIXEL);
}

void LazySnapping::SetLazySnappingMethod(LazySnappingType lst) {
  m_lazy_type = lst;
}

#if 0
double LazySnapping::GetEnergyRegionItem(int colour, Scene scn) {
  double sub_dist = GetColourDistence(colour, *m_k_means_sub);
  double bck_dist = GetColourDistence(colour, *m_k_means_bck);
  double sum_dist = sub_dist + bck_dist;
  if (scn == SUBJECT) {
    return sub_dist / sum_dist;
  } else if (scn == BACKGROUND) {
    return bck_dist / sum_dist;
  } else {
    printf("error: the scene is wrong\n");
    return 0;
  }
}

double LazySnapping::GetEnergyBoundryItem(int colour, int near_colour, Direction drc) {
  return LAMDA / (COLOUR_DIST(colour, near_colour) + 0.01);
}

void LazySnapping::SegmentImageByGraph(const GrapyType& graph, ImageData<int>* marked_image) {
  assert(marked_image != NULL && marked_image->IsEmpty() == false);
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      if (graph.what_segment(index) == GraphType::SOURCE) {
        SET_PIXEL(marked_image, index, SUBJECT);
      } else {
        SET_PIXEL(marked_image, index, BACKGROUND);
      }
    }
  }
}
#endif
