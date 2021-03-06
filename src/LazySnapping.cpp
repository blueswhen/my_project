// Copyright sxniu 2014-9
#include "include/LazySnapping.h"

#include <vector>
#include <map>
#include <unordered_map>
#include <stdio.h>
#include <float.h>
#include <assert.h>
#include <opencv/highgui.h>
#include <pthread.h>

#include "include/ImageData.h"
#include "include/utils.h"
#include "include/colour.h"
#include "include/CountTime.h"
#include "include/WatershedRegion.h"
#include "include/Square.h"
#include "include/Lines.h"
#include "include/ui.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/Graph.h"
#include "include/IGraph.h"
#include "include/IFGraph.h"
#include "include/gcgraph.hpp"
#include "include/maxflow-v3.03/graph.h"
// #include "include/ibfs/ibfs.h"
// #include "include/ibfs/iibfs.h"

#define START_GRADIENT 10
// START_MARK_NUM must be positive number
#define START_MARK_NUM 100
#define K_NUM 60
#define ITER 10
#define LAMDA 50
#define HUGE_MAX 50000

typedef Graph<double, double, double> GraphType;
typedef double (*EPF)(int src_node_colour, int dst_node_colour);

using namespace cv;

inline double GetColourDistence(double c_mean[3],
                                const std::vector<std::vector<double> >& k_means) {
  double min = HUGE_MAX;
  int n = k_means.size();
  for (int i = 0; i < n; ++i) {
    double diff = THREE_DIM_DIST_SQUARE(c_mean, k_means[i]);
    if (diff < min) {
      min = diff;
    }
  }
  if (!min) {
    min += EPSILON;
  }
  return min;
}

inline double GetColourDistence(const int& colour,
                                const std::vector<std::vector<double> >& k_means) {
  int rgb[3] = GET_THREE_COORDINATE(colour);
  double d_rgb[3];
  d_rgb[0] = static_cast<double>(rgb[0]);
  d_rgb[1] = static_cast<double>(rgb[1]);
  d_rgb[2] = static_cast<double>(rgb[2]);
  return GetColourDistence(d_rgb, k_means);
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
      e1[0] = HUGE_MAX;
      e1[1] = 0;
    } else {
      e1[0] = 0;
      e1[1] = HUGE_MAX;
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

  // graph.maxflow();

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

double EdgePunishItem(int src_node_colour, int dst_node_colour) {
  return LAMDA / (COLOUR_DIST(src_node_colour, dst_node_colour) + 0.01);
  // return LAMDA * exp (-0.03*COLOUR_DIST(src_node_colour, dst_node_colour));
}

void GraphCutBasePixel(const std::vector<std::vector<double> >& k_means_sub,
                       const std::vector<std::vector<double> >& k_means_bck,
                       const ImageData<int>& source_image,
                       ImageData<int>* marked_image,
                       std::unordered_map<int, int>* graph_vtx_map) {
#define BUILD_VTX(index) \
  (graph_vtx_map != NULL ? (*graph_vtx_map)[index] : index)

#define IS_BUILD_EDGE(index) \
  (graph_vtx_map != NULL && graph_vtx_map->find(index) != graph_vtx_map->end())

  // CountTime ct_;
  double time = 0;
  int width = source_image.GetWidth();
  int height = source_image.GetHeight();
  if (marked_image->IsEmpty()) {
    marked_image->CreateEmptyImage(width, height);
  }
  int vtx_count = width * height;
  int edge_count = 4 * vtx_count - 3 * (width + height) + 2;

#define ADD_EDGE 1
#define MY_MAXFLOW 1
#define BK_MAXFLOW 0
#define IB_MAXFLOW 0
#define MY_IB_MAXFLOW 0
#define OPENCV_MAXFLOW 0
#define IGRAPH 0
#define IFGRAPH 0

#if MY_MAXFLOW
  user::Graph<double> mygraph(vtx_count, 2 * edge_count);
  // user::Graph<double> mygraph(vtx_count, width, height, EdgePunishItem);
#endif

#if BK_MAXFLOW
  GraphType bkgraph(vtx_count, edge_count);
  // GraphType bkgraph(vtx_count, width, height, EdgePunishItem);
#endif

#if OPENCV_MAXFLOW
  GCGraph<double> graph;
  graph.create(vtx_count, edge_count);
#elif IB_MAXFLOW
  IBFSGraph graph(IBFSGraph::IB_INIT_FAST);
  // IBFSGraph graph;
  graph.initSize(vtx_count, edge_count);
  // graph.initSize(vtx_count, width, height, EdgePunishItem);
#endif

#if MY_IB_MAXFLOW
  IIBFSGraph ibgraph;
  ibgraph.initSize(vtx_count, width, height, EdgePunishItem);
#endif

#if IGRAPH
  IGraph<double, EPF> igraph(vtx_count, width, height, EdgePunishItem, marked_image);
#endif

#if IFGRAPH
  IFGraph<double, EPF> ifgraph(vtx_count, width, height, EdgePunishItem, marked_image);
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
      } else if (marked_colour == SUBJECT || (marked_colour & RIGHT_HALF) == OLD_SUB) {
        e1[0] = HUGE_MAX;
        e1[1] = 0;
      } else {
        e1[0] = 0;
        e1[1] = HUGE_MAX;
      }
      int vtx0 = BUILD_VTX(index);
#if MY_MAXFLOW
      mygraph.AddNode(vtx0, e1[0], e1[1]);
      // mygraph.AddNode(vtx0, e1[0], e1[1], colour);
      // mygraph.AddActiveNodes(x, y);
#endif

#if BK_MAXFLOW
      bkgraph.add_node();
      bkgraph.add_tweights(vtx0, e1[0], e1[1]);
      // bkgraph.add_tweights(vtx0, e1[0], e1[1], colour);
      // bkgraph.AddActiveNodes(x, y);
#endif
#if OPENCV_MAXFLOW
      graph.addVtx();
      graph.addTermWeights(index, e1[0], e1[1]);
#elif IB_MAXFLOW
      graph.addNode(vtx0, e1[0], e1[1]);
      // graph.addNode(vtx0, e1[0], e1[1], colour);
      // graph.AddActiveNodes(x, y);
#endif

#if MY_IB_MAXFLOW
      ibgraph.addNode(vtx0, e1[0], e1[1], colour);
      ibgraph.AddActiveNodes(x, y);
#endif

#if IGRAPH
      igraph.AddNode(vtx0, e1[0], e1[1], colour);
      igraph.AddActiveNodes(x, y);
#endif
#if IFGRAPH
      ifgraph.AddNode(vtx0, e1[0], e1[1], colour);
      ifgraph.AddActiveNodes(x, y);
#endif

  // ct_.ContBegin();
#if ADD_EDGE
      // 8 neighbours
      if ((graph_vtx_map == NULL && x > 0) || IS_BUILD_EDGE(index - 1)) {
        int vtx1 = BUILD_VTX(index - 1);
        int near_colour = GET_PIXEL(&source_image, index - 1);
        double e2 = lamda / (COLOUR_DIST(colour, near_colour) + 0.01);
        // double e2 = lamda * exp(-0.03*COLOUR_DIST(colour, near_colour));
#if MY_MAXFLOW
        mygraph.AddEdge(vtx0, vtx1, e2);
#endif

#if BK_MAXFLOW
        bkgraph.add_edge(vtx0, vtx1, e2, e2);
#endif
#if OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#elif IB_MAXFLOW
        graph.addEdge(vtx0, vtx1, e2, e2);
#endif
      }
      if ((graph_vtx_map == NULL && x > 0 && y > 0) || IS_BUILD_EDGE(index - width - 1)) {
        int vtx1 = BUILD_VTX(index - width - 1);
        int near_colour = GET_PIXEL(&source_image, index - width - 1);
        double e2 = lamda_div_sqrt2 / (COLOUR_DIST(colour, near_colour) + 0.01);
        // double e2 = lamda_div_sqrt2 * exp(-0.03*COLOUR_DIST(colour, near_colour));
#if MY_MAXFLOW
        mygraph.AddEdge(vtx0, vtx1, e2);
#endif

#if BK_MAXFLOW
        bkgraph.add_edge(vtx0, vtx1, e2, e2);
#endif
#if OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#elif IB_MAXFLOW
        graph.addEdge(vtx0, vtx1, e2, e2);
#endif
      }
      if ((graph_vtx_map == NULL && y > 0) || IS_BUILD_EDGE(index - width)) {
        int vtx1 = BUILD_VTX(index - width);
        int near_colour = GET_PIXEL(&source_image, index - width);
        double e2 = lamda / (COLOUR_DIST(colour, near_colour) + 0.01);
        // double e2 = lamda * exp(-0.03*COLOUR_DIST(colour, near_colour));
#if MY_MAXFLOW
        mygraph.AddEdge(vtx0, vtx1, e2);
#endif

#if BK_MAXFLOW
        bkgraph.add_edge(vtx0, vtx1, e2, e2);
#endif
#if OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#elif IB_MAXFLOW
        graph.addEdge(vtx0, vtx1, e2, e2);
#endif
      }
      if ((graph_vtx_map == NULL && x < width - 1 && y > 0) || IS_BUILD_EDGE(index - width + 1)) {
        int vtx1 = BUILD_VTX(index - width + 1);
        int near_colour = GET_PIXEL(&source_image, index - width + 1);
        double e2 = lamda_div_sqrt2 / (COLOUR_DIST(colour, near_colour) + 0.01);
        // double e2 = lamda_div_sqrt2 * exp(-0.03*COLOUR_DIST(colour, near_colour));
#if MY_MAXFLOW
        mygraph.AddEdge(vtx0, vtx1, e2);
#endif

#if BK_MAXFLOW
        bkgraph.add_edge(vtx0, vtx1, e2, e2);
#endif
#if OPENCV_MAXFLOW
        graph.addEdges(vtx0, vtx1, e2, e2);
#elif IB_MAXFLOW
        graph.addEdge(vtx0, vtx1, e2, e2);
#endif
      }
#endif
// ct_.ContEnd();
// time += ct_.ContResult();
    }
  }
  // printf("time = %f\n", 1000 * time);

#if 1
CountTime ct;
#if MY_MAXFLOW
// ct.ContBegin();
  mygraph.Init();
  mygraph.MaxFlow();
// ct.ContEnd();
// ct.PrintTime();
#endif

#if BK_MAXFLOW
ct.ContBegin();
  bkgraph.maxflow();
ct.ContEnd();
ct.PrintTime();
#endif
#if OPENCV_MAXFLOW
// ct.ContBegin();
  graph.maxFlow();
// ct.ContEnd();
// ct.PrintTime();
#elif IB_MAXFLOW
ct.ContBegin();
  graph.initGraph();
  graph.computeMaxFlow();
ct.ContEnd();
ct.PrintTime();
#endif

#if MY_IB_MAXFLOW
  ibgraph.computeMaxFlow();
#endif

#if IGRAPH
// ct.ContBegin();
  igraph.MaxFlow();
// ct.ContEnd();
// ct.PrintTime();
#endif

#if IFGRAPH
// ct.ContBegin();
  ifgraph.MaxFlow();
// ct.ContEnd();
// ct.PrintTime();
#endif
#endif

#if 0
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int vtx0 = BUILD_VTX(y * width + x);
      if ((bkgraph.what_segment(vtx0) == GraphType::SOURCE) !=
          mygraph.IsBelongToSource(vtx0)) {
      // if ((bkgraph.what_segment(vtx0) == GraphType::SOURCE) !=
      //     ifgraph.IsBelongToSource(vtx0)) {
        printf("error pixel\n");
      }
    }
  }
#endif

#if 0
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int vtx0 = BUILD_VTX(y * width + x);
      // if (graph.isNodeOnSrcSide(vtx0) != ibgraph.isNodeOnSrcSide(vtx0)) {
      if (igraph.IsBelongToSource(vtx0) != graph.isNodeOnSrcSide(vtx0)) {
        printf("error pixel, x = %d, y = %d\n", x, y);
      }
    }
  }
#endif

#if 1
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_colour = GET_PIXEL(marked_image, index);
      if (marked_colour != IGNORED && (marked_colour & RIGHT_HALF) != OLD_SUB &&
          (marked_colour & RIGHT_HALF) != OLD_BCK) {
        int vtx0 = BUILD_VTX(index);
#if MY_MAXFLOW
        if (mygraph.IsBelongToSource(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#endif

#if BK_MAXFLOW
        if (bkgraph.what_segment(vtx0) == GraphType::SOURCE) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#endif
#if OPENCV_MAXFLOW
        if (graph.inSourceSegment(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#elif IB_MAXFLOW 
        if (graph.isNodeOnSrcSide(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#endif
#if MY_IB_MAXFLOW
        if (ibgraph.isNodeOnSrcSide(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#elif IGRAPH
        if (igraph.IsBelongToSource(vtx0)) {
          SET_PIXEL(marked_image, index, SUBJECT);
        } else {
          SET_PIXEL(marked_image, index, BACKGROUND);
        }
#elif IFGRAPH
        if (ifgraph.IsBelongToSource(vtx0)) {
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

void* fun(void* arg) {
  LazySnapping* ls = (LazySnapping*)arg;
  ls->DoPartition();
  return NULL;
}

LazySnapping::LazySnapping(SegmentationData* sd, UserInput* usr_input)
  : Segmentation(sd, usr_input)
  , m_lazy_type(PIXEL) {
    m_graph_vtx_map = new std::unordered_map<int, int>();
  }

LazySnapping::~LazySnapping() {
  if (m_graph_vtx_map != NULL) {
    delete m_graph_vtx_map;
    m_graph_vtx_map = NULL;
  }
}

void LazySnapping::InitMarkedImage(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  std::vector<UserInput::LinePoint>* sub_mark_points = uip->GetSubjectPoints();
  std::vector<UserInput::LinePoint>* bck_mark_points = uip->GetBackgroundPoints();
  assert(sub_mark_points != NULL && bck_mark_points != NULL);
  ImageData<int>* marked_image = sd->GetMarkedImage();
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int col = GET_PIXEL(marked_image, y * width + x);
      SET_PIXEL(marked_image, y * width + x, UNDEFINE);
    }
  }
  for (int i = 0; i < sub_mark_points->size(); ++i) {
    SET_PIXEL(marked_image, (*sub_mark_points)[i].index, SUBJECT);
  }
  for (int i = 0; i < bck_mark_points->size(); ++i) {
    SET_PIXEL(marked_image, (*bck_mark_points)[i].index, BACKGROUND);
  }
}

void LazySnapping::UpdateSceneVectorFromSourceImage() {
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  ImageData<int>* src_image = m_sd->GetSourceImageBck();
  ImageData<int>* marked_image = m_sd->GetMarkedImage();
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  std::vector<UserInput::LinePoint>* sub_mark_points = m_usr_input->GetSubjectPoints();
  std::vector<UserInput::LinePoint>* bck_mark_points = m_usr_input->GetBackgroundPoints();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int ui_col = GET_PIXEL(ui_image, index) & RIGHT_HALF;
      int src_col = GET_PIXEL(src_image, index);
      if (ui_col == m_sd->GetSubjectColour()) {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = src_col;
        sub_mark_points->push_back(lp);
      } else if (ui_col == m_sd->GetBackgroundColour()) {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = src_col;
        bck_mark_points->push_back(lp);
      }
    }
  }
}

void LazySnapping::Cut(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  Segmentation::RemoveLastResult(sd);
  // get user input data, these code needn't to change
  // whatever the situation is lines or square or lasso
  ImageData<int>* ui_image = sd->GetSourceImage();
  const ImageData<int>* source_image = sd->GetSourceImageBck();
  int subject_colour = sd->GetSubjectColour();
  int background_colour = sd->GetBackgroundColour();
  ImageData<int>* marked_image = sd->GetMarkedImage();
  std::vector<UserInput::LinePoint>* sub_mark_points = m_usr_input->GetSubjectPoints();
  std::vector<UserInput::LinePoint>* bck_mark_points = m_usr_input->GetBackgroundPoints();
  std::vector<int> sub_mark_value(sub_mark_points->size());
  std::vector<int> bck_mark_value(bck_mark_points->size());
  for (int i = 0; i < sub_mark_points->size(); ++i) {
    sub_mark_value[i] = (*sub_mark_points)[i].value;
  }
  for (int i = 0; i < bck_mark_points->size(); ++i) {
    bck_mark_value[i] = (*bck_mark_points)[i].value;
  }

  std::vector<std::vector<double> > k_means_sub(K_NUM, std::vector<double>(3));
  std::vector<std::vector<double> > k_means_bck(K_NUM, std::vector<double>(3));
  std::vector<int> cluster_vec_sub(sub_mark_value.size());
  std::vector<int> cluster_vec_bck(bck_mark_value.size());
  utils::Kmeans(sub_mark_value, &cluster_vec_sub, &k_means_sub,
                K_NUM, ITER, ui_image->GetRandomSeed());
  utils::Kmeans(bck_mark_value, &cluster_vec_bck, &k_means_bck,
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
    // WatershedRegionGroup wrg(*source_image, *marked_image,
    //                          *sub_mark_index, *bck_mark_index,
    //                          region_count, START_MARK_NUM);
    // GraphCutBaseWatershed(k_means_sub, k_means_bck, &wrg);
    // utils::ExtractContourLine(wrg, ui_image, marked_image);
  } else {
  // CountTime ct;
  // ct.ContBegin();
    GraphCutBasePixel(k_means_sub, k_means_bck, *source_image, marked_image, m_graph_vtx_map);
  // ct.ContEnd();
  // ct.PrintTime();
    utils::ExpandArea(marked_image, Lines::GetMarkedAreaId());
    utils::ExtractContourLine(m_sd);
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

void LazySnapping::MakeGraphVtx(const ImageData<int>& marked_image) {
  int width = marked_image.GetWidth();
  int height = marked_image.GetHeight();
  int i = 0;
  assert(m_graph_vtx_map != NULL);
  m_graph_vtx_map->clear();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&marked_image, index);
      if (colour != IGNORED) {
        (*m_graph_vtx_map)[index] = i++;
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
  std::vector<UserInput::LinePoint>* sub_mark_points = m_usr_input->GetSubjectPoints();
  std::vector<UserInput::LinePoint>* bck_mark_points = m_usr_input->GetBackgroundPoints();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_colour = GET_PIXEL(marked_image, index);
      int src_colour = GET_PIXEL(src_image, index);
      if (marked_colour == SUBJECT) {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = src_colour;
        sub_mark_points->push_back(lp);
      } else if (marked_colour == BACKGROUND) {
        UserInput::LinePoint lp;
        lp.index = index;
        lp.value = src_colour;
        bck_mark_points->push_back(lp);
      }
    }
  }
}

bool LazySnapping::SetSquareAreaForMarkedImage(int cen_x, int cen_y) {
  assert(m_sd != NULL);
  ImageData<int>* scr_image = m_sd->GetSourceImageBck();
  ImageData<int>* marked_image = m_sd->GetMarkedImage();
  ImageData<int>* ui_image = m_sd->GetSourceImage();
  int width = scr_image->GetWidth();
  int height = scr_image->GetHeight();
  auto& subject_points = *m_usr_input->GetSubjectPoints();
  auto& background_points = *m_usr_input->GetBackgroundPoints();

  double square_radius = Lines::GetSquareRadius();
  int leftup_x = std::max(static_cast<int>(cen_x - square_radius), 0);
  int rightdown_x = std::min(static_cast<int>(cen_x + square_radius), width - 1);
  int leftup_y = std::max(static_cast<int>(cen_y - square_radius), 0);
  int rightdown_y = std::min(static_cast<int>(cen_y + square_radius), height - 1);
  // set all pixels in marked_image IGNORED
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int col = GET_PIXEL(marked_image, index);
      if ((col & RIGHT_HALF) != OLD_SUB && (col & RIGHT_HALF) != OLD_BCK) {
        SET_PIXEL(marked_image, y * width + x, IGNORED);
      }
    }
  }
  // set UNDEFINE
  if (m_usr_input->GetUsrInputScene() == SUBJECT) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        int col = GET_PIXEL(marked_image, index);
        if (y >= leftup_y && y <= rightdown_y && x >= leftup_x && x <= rightdown_x) {
          if ((col & RIGHT_HALF) != OLD_SUB) {
            SET_PIXEL(marked_image, index, UNDEFINE);
          }
        }
        int ui_col = GET_PIXEL(ui_image, index);
        if ((ui_col & RIGHT_HALF) == m_sd->GetBackgroundColour()) {
          UserInput::LinePoint lp;
          lp.index = index;
          lp.value = GET_PIXEL(scr_image, index);
          background_points.push_back(lp);
        }
      }
    }
  } else {
    // get sub_area_ids
    std::vector<int> sub_area_ids;
    for (auto rit = background_points.rbegin(); rit != background_points.rend(); ++rit) {
      if (rit->index == -1) {
        sub_area_ids.push_back(rit->value);
        assert(rit == background_points.rbegin());
        background_points.pop_back();
      } else {
        break;
      }
    }
    if (sub_area_ids.empty()) {
      return false;
    }
    // set UNDEFINE according to the area ids
    assert(background_points.rbegin()->index != -1);
    bool assert_true = false;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        int mark_col = GET_PIXEL(marked_image, index);
        int col = GET_PIXEL(ui_image, index);
        if (y >= leftup_y && y <= rightdown_y && x >= leftup_x && x <= rightdown_x) {
          for (int i = 0; i < sub_area_ids.size(); ++i) {
            if (mark_col == sub_area_ids[i]) {
              SET_PIXEL(marked_image, index, UNDEFINE);
              assert_true = true;
            }
          }
          if ((col & RIGHT_HALF) == m_sd->GetSubjectColour()) {
            UserInput::LinePoint lp;
            lp.index = index;
            lp.value = GET_PIXEL(scr_image, index);
            subject_points.push_back(lp);
          }
        }
      }
    }
    assert(assert_true);
  }
  Scene usr_scn = m_usr_input->GetUsrInputScene();
  int subject_colour = usr_scn == SUBJECT ? (OLD_SUB + Lines::GetMarkedAreaId()) : SUBJECT;
  int background_colour = usr_scn == SUBJECT ? BACKGROUND : (OLD_BCK + Lines::GetMarkedAreaId());
  for (int i = 0; i < subject_points.size(); ++i) {
    int mark_col = GET_PIXEL(marked_image, subject_points[i].index);
    if ((mark_col & RIGHT_HALF) != OLD_SUB) {
      SET_PIXEL(marked_image, subject_points[i].index, subject_colour);
    }
  }
  for (int i = 0; i < background_points.size(); ++i) {
    int mark_col = GET_PIXEL(marked_image, background_points[i].index);
    if ((mark_col & RIGHT_HALF) != OLD_BCK) {
      if (usr_scn == SUBJECT || mark_col == UNDEFINE) {
        SET_PIXEL(marked_image, background_points[i].index, background_colour);
      }
    }
  }
  MakeGraphVtx(*marked_image);
  return true;
}

void LazySnapping::DoPartition() {
  CountTime ct;
  ct.ContBegin();

  SegmentationWithoutCoarsen();
  // SegmentationWithCoarsen();

  ct.ContEnd();
  ct.PrintTime();
}

#define DRAW_LINE_TEST

void LazySnapping::DoLeftButtonDown(int x, int y) {
  // UpdateSceneVectorFromSourceImage();
  m_usr_input->SetUsrInputScene(SUBJECT);
#ifdef DRAW_LINE_TEST 
  m_usr_input->Reset();
  m_usr_input->DrawFirstPointForSub(x, y);
#else
  DoPartition();
#endif
#if 0
  pthread_t id;
  pthread_create(&id, NULL, fun, this);
#endif
}

void LazySnapping::DoRightButtonDown(int x, int y) {
  m_usr_input->SetUsrInputScene(BACKGROUND);
  m_usr_input->Reset();
  m_usr_input->DrawFirstPointForBck(x, y);
}

void LazySnapping::DoLeftMouseMove(int x, int y) {
  m_usr_input->DrawSubjectBegin(x, y);
  if (!m_usr_input->IsCut() || !SetSquareAreaForMarkedImage(x, y)) {
    Segmentation::RemoveLastResult(m_sd);
    utils::ExtractContourLine(m_sd);
    return;
  }
  Cut(m_sd, m_usr_input);
  // DoPartition();
}

void LazySnapping::DoRightMouseMove(int x, int y) {
  m_usr_input->DrawBackgroundBegin(x, y);
  if (!m_usr_input->IsCut() || !SetSquareAreaForMarkedImage(x, y)) {
    Segmentation::RemoveLastResult(m_sd);
    utils::ExtractContourLine(m_sd);
    return;
  }
  Cut(m_sd, m_usr_input);
}

void LazySnapping::DoLeftButtonUp(int x, int y) {
#ifdef DRAW_LINE_TEST 
  m_usr_input->DrawSubjectFinish(x, y);
  // std::string input_name = m_usr_input->GetImageName() + "_input.bmp";
  // utils::SaveImage(input_name.c_str(), *(m_sd->GetSourceImage()));
  // DoPartition();
#endif
}

void LazySnapping::DoRightButtonUp(int x, int y) {
  m_usr_input->DrawBackgroundFinish(x, y);
}

void LazySnapping::ResetUserInput() {
  Segmentation::ResetUserInput();
  SetLazySnappingMethod(LazySnapping::PIXEL);
  if (m_graph_vtx_map != NULL) {
    delete m_graph_vtx_map;
    m_graph_vtx_map = new std::unordered_map<int, int>();
  }
}

void LazySnapping::SetLazySnappingMethod(LazySnappingType lst) {
  m_lazy_type = lst;
}
