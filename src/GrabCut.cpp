// Copyright sxniu 2014-9
#include "include/GrabCut.h"

#include <vector>
#include <map>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <assert.h>
#include <opencv/highgui.h>

#include "include/ImageData.h"
#include "include/utils.h"
#include "include/colour.h"
#include "include/CountTime.h"
#include "include/WatershedRegion.h"
#include "include/ui.h"
#include "include/Gmm.h"
#include "include/IGraph.h"
// #include "include/Graph.h"
#include "include/maxflow-v3.03/block.h"
#include "include/maxflow-v3.03/graph.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"

typedef Graph<double, double, double> GraphType;
// k means
#define ITER 10

// grab cut
#define ITER_COUNT 1

using namespace cv;

enum GrabScene {
  BCK = 0,
  SUB = 1,
  PR_BCK = 2,
  PR_SUB = 3 
};

void GrabCut::DoLeftButtonDown(int x, int y) {
  m_usr_input->SetUsrInputScene(SUBJECT);
  m_usr_input->DrawFirstPointForSub(x, y);
}

void GrabCut::DoRightButtonDown(int x, int y) {
  m_usr_input->SetUsrInputScene(BACKGROUND);
  m_usr_input->DrawFirstPointForBck(x, y);
}

void GrabCut::DoLeftMouseMove(int x, int y) {
  m_usr_input->DrawSubjectBegin(x, y);
}

void GrabCut::DoRightMouseMove(int x, int y) {
  m_usr_input->DrawBackgroundBegin(x, y);
}

void GrabCut::DoLeftButtonUp(int x, int y) {
  m_usr_input->DrawSubjectFinish(x, y);
  // std::string input_name = m_usr_input->GetImageName() + "_sr_input.bmp";
  // utils::SaveImage(input_name.c_str(), *(m_sd->GetSourceImage()));
  DoPartition();
}

void GrabCut::DoRightButtonUp(int x, int y) {
  m_usr_input->DrawBackgroundFinish(x, y);
  DoPartition();
}

/*
  Initialize Gmm background and foreground models using kmeans algorithm.
*/
void InitGmms(const ImageData<int>& image, const std::vector<int>& subject_value,
              const std::vector<int>& background_value,
              Gmm* background_gmm, Gmm* subject_gmm) {
  assert(background_gmm != NULL && subject_gmm != NULL);
  assert(subject_value.size() != 0 && background_value.size() != 0);

  std::vector<int> subject_labels(subject_value.size());
  std::vector<int> background_labels(background_value.size());

  utils::Kmeans(subject_value, &subject_labels, NULL,
                Gmm::m_components_count, ITER, image.GetRandomSeed());
  utils::Kmeans(background_value, &background_labels, NULL,
                Gmm::m_components_count, ITER, image.GetRandomSeed());

  subject_gmm->InitLearning();
  for(int i = 0; i < subject_value.size(); ++i) {
    subject_gmm->AddSample(subject_labels[i], subject_value[i]);
  }
  subject_gmm->EndLearning();

  background_gmm->InitLearning();
  for(int i = 0; i < background_value.size(); ++i) {
    background_gmm->AddSample(background_labels[i], background_value[i]);
  }
  background_gmm->EndLearning();
}

/*
  Assign Gmms components for each pixel.
*/
void AssignGmmsComponents(const ImageData<int>& image, const ImageData<int>& marked_image,
                          Gmm* background_gmm, Gmm* subject_gmm,
                          std::vector<uchar>* comp_idxs) {
  assert(background_gmm != NULL && subject_gmm != NULL && comp_idxs != NULL);
  int width = image.GetWidth();
  int height = image.GetHeight();
  assert(comp_idxs->size() == width * height);
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&image, index);
      int marked_colour = GET_PIXEL(&marked_image, index);
      (*comp_idxs)[index] = marked_colour == BCK || marked_colour == PR_BCK ?
                            background_gmm->WhichComponent(colour) :
                            subject_gmm->WhichComponent(colour);
    }
  }
}

/*
  Learn Gmms parameters.
*/
void LearnGmms(const ImageData<int>& image, const ImageData<int>& marked_image,
               const std::vector<uchar>& comp_idxs,
               Gmm* background_gmm, Gmm* subject_gmm) {
  background_gmm->InitLearning();
  subject_gmm->InitLearning();
  int width = image.GetWidth();
  int height = image.GetHeight();
  assert(comp_idxs.size() == width * height);
  for(int ci = 0; ci < Gmm::m_components_count; ++ci) {
    for(int y = 0; y < height; ++y) {
      for(int x = 0; x < width; ++x) {
        int index = y * width + x;
        int marked_colour = GET_PIXEL(&marked_image, index);
        int colour = GET_PIXEL(&image, index);
        if(comp_idxs[index] == ci) {
          if(marked_colour == BCK || marked_colour == PR_BCK) {
            background_gmm->AddSample(ci, colour);
          } else {
            subject_gmm->AddSample(ci, colour);
          }
        }
      }
    }
  }
  background_gmm->EndLearning();
  subject_gmm->EndLearning();
}

class EdgePunishItem {
 public:
  EdgePunishItem(double gamma, double beta)
    : m_gamma(gamma)
    , m_beta(beta) {}
  double operator()(int src_node_colour, int dst_node_colour) {
    return m_gamma * exp(-m_beta * COLOUR_DIST_SQUARE(src_node_colour, dst_node_colour));
  }
 private:
  double m_gamma;
  double m_beta;
};

void GraphCutWithGrab(const ImageData<int>& image, ImageData<int>* marked_image,
                      const Gmm& background_gmm, const Gmm& subject_gmm,
                      double lambda, double beta, double gamma,
                      std::map<int, int>* graph_vtx_map) {
#define BUILD_VTX(index) \
({ \
  graph_vtx_map != NULL ? (*graph_vtx_map)[index] : index; \
})

#define IS_BUILD_EDGE(index) \
  (graph_vtx_map != NULL && graph_vtx_map->find(index) != graph_vtx_map->end())

  int width = image.GetWidth();
  int height = image.GetHeight();
  int vtx_count = width * height;
  int edge_count = 2 * (4 * vtx_count - 3 * (width + height) + 2);

#define B_MAXFLOW 1
#define IGRAPH 0

#if B_MAXFLOW
  GraphType graph(vtx_count, edge_count);
#endif

  EdgePunishItem epi(gamma, beta);
#if IGRAPH
  IGraph<double, EdgePunishItem> igraph(vtx_count, width, height, epi, marked_image);
#endif

  const double gammaDivSqrt2 = gamma / std::sqrt(2.0f);
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(&image, index);
      int marked_colour = GET_PIXEL(marked_image, index);
      if (marked_colour == IGNORED) {
        continue;
      }

      // add node
      // e1[0] is background, e1[1] is subject
      double e1[2] = {0, 0};
      if (marked_colour == PR_SUB || marked_colour == PR_BCK) {
        e1[0] = -log(background_gmm.SceneProbability(colour));
        e1[1] = -log(subject_gmm.SceneProbability(colour));
      } else if (marked_colour == BCK) {
        e1[0] = 0;
        e1[1] = lambda;
      } else {
        e1[0] = lambda;
        e1[1] = 0;
      }
      int vtx0 = BUILD_VTX(index);
#if B_MAXFLOW
      graph.add_node();
      graph.add_tweights(vtx0, e1[0], e1[1]);
#endif
#if IGRAPH
      igraph.AddNode(vtx0, e1[0], e1[1], colour);
      igraph.AddActiveNodes(x, y);
#endif

#if B_MAXFLOW
      // 8 neighbours
      if ((graph_vtx_map == NULL && x > 0) || IS_BUILD_EDGE(index - 1)) {
        int colour_arr = GET_PIXEL(&image, y * width + x - 1);
        double e2 = gamma * exp(-beta * COLOUR_DIST_SQUARE(colour, colour_arr));
        int vtx1 = BUILD_VTX(index - 1);
        graph.add_edge(vtx0, vtx1, e2, e2);
      }
      if ((graph_vtx_map == NULL && x > 0 && y > 0) || IS_BUILD_EDGE(index - width - 1)) {
        int colour_arr = GET_PIXEL(&image, (y - 1) * width + x - 1);
        double e2 = gammaDivSqrt2 * exp(-beta * COLOUR_DIST_SQUARE(colour, colour_arr));
        int vtx1 = BUILD_VTX(index -width - 1);
        graph.add_edge(vtx0, vtx1, e2, e2);
      }
      if ((graph_vtx_map == NULL && y > 0) || IS_BUILD_EDGE(index - width)) {
        int colour_arr = GET_PIXEL(&image, (y - 1) * width + x);
        double e2 = gamma * exp(-beta * COLOUR_DIST_SQUARE(colour, colour_arr));
        int vtx1 = BUILD_VTX(index - width);
        graph.add_edge(vtx0, vtx1, e2, e2);
      }
      if ((graph_vtx_map == NULL && x < width - 1 && y > 0) || IS_BUILD_EDGE(index - width + 1)) {
        int colour_arr = GET_PIXEL(&image, (y - 1) * width + x + 1);
        double e2 = gammaDivSqrt2 * exp(-beta * COLOUR_DIST_SQUARE(colour, colour_arr));
        int vtx1 = BUILD_VTX(index - width + 1);
        graph.add_edge(vtx0, vtx1, e2, e2);
      }
#endif
    }
  }

#if B_MAXFLOW
  // CountTime ct;
  // ct.ContBegin();

  graph.maxflow();

  // ct.ContEnd();
  // ct.PrintTime();
#endif
#if IGRAPH
  // CountTime ct;
  // ct.ContBegin();

  igraph.MaxFlow();

  // ct.ContEnd();
  // ct.PrintTime();
#endif

#if 0
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int vtx0 = BUILD_VTX(y * width + x);
      if ((graph.what_segment(vtx0) == GraphType::SOURCE) !=
          igraph.IsBelongToSource(vtx0)) {
        printf("error pixel\n");
      }
    }
  }
#endif

#if 1
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int marked_colour = GET_PIXEL(marked_image, index);
      if (marked_colour == PR_SUB || marked_colour == PR_BCK) {
        int vtx0 = BUILD_VTX(index);
#if B_MAXFLOW
        if (graph.what_segment(vtx0) == GraphType::SOURCE) {
          SET_PIXEL(marked_image, index, PR_SUB);
        } else {
          SET_PIXEL(marked_image, index, PR_BCK);
        }
#endif
#if IGRAPH 
        if (igraph.IsBelongToSource(vtx0)) {
          SET_PIXEL(marked_image, index, PR_SUB);
        } else {
          SET_PIXEL(marked_image, index, PR_BCK);
        }
#endif
      }
    }
  }
#endif
}

void CreateFinalMarkedImage(ImageData<int>* marked_image) {
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int value = GET_PIXEL(marked_image, index);
      if ((value & 1) == 1) {
        SET_PIXEL(marked_image, index, WHITE);
      } else {
        SET_PIXEL(marked_image, index, BLACK);
      }
    }
  }
}

void GrabCut::DoPartition() {
  // CountTime ct;
  // ct.ContBegin();

  SegmentationWithoutCoarsen();
  // SegmentationWithCoarsen();

  // ct.ContEnd();
  // ct.PrintTime();
}

void GrabCut::InitMarkedImage(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  std::vector<UserInput::LinePoint>* subject_points = uip->GetSubjectPoints();
  std::vector<UserInput::LinePoint>* background_points = uip->GetBackgroundPoints();
  assert(subject_points!= NULL && background_points != NULL);
  ImageData<int>* marked_image = sd->GetMarkedImage();

  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      SET_PIXEL(marked_image, index, PR_SUB);
    }
  }
  for (int i = 0; i < background_points->size(); ++i) {
    SET_PIXEL(marked_image, (*background_points)[i].index, BCK);
  }
  if(subject_points->size() + background_points->size() != width * height) {
    for (int i = 0; i < subject_points->size(); ++i) {
      SET_PIXEL(marked_image, (*subject_points)[i].index, SUB);
    }
  }
}

void GrabCut::Cut(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  Segmentation::RemoveLastResult(sd);
  // get user input data, these code needn't to change
  // whatever the situation is lines or square or lasso
  int subject_colour = sd->GetSubjectColour();
  int background_colour = sd->GetBackgroundColour();
  ImageData<int>* ui_image = sd->GetSourceImage();
  ImageData<int>* marked_image = sd->GetMarkedImage();
  const ImageData<int>* source_image = sd->GetSourceImageBck();
  std::vector<UserInput::LinePoint>* sub_mark_points = uip->GetSubjectPoints();
  std::vector<UserInput::LinePoint>* bck_mark_points = uip->GetBackgroundPoints();

  assert(ui_image != NULL && source_image != NULL && marked_image != NULL &&
         ui_image->IsEmpty() == false && source_image->IsEmpty() == false &&
         marked_image->IsEmpty() == false);

  Gmm background_gmm;
  Gmm subject_gmm;
  std::vector<uchar> comp_idxs(source_image->GetWidth() * source_image->GetHeight());

  std::vector<int> sub_mark_value(sub_mark_points->size());
  std::vector<int> bck_mark_value(bck_mark_points->size());
  for (int i = 0; i < sub_mark_points->size(); ++i) {
    sub_mark_value[i] = (*sub_mark_points)[i].value;
  }
  for (int i = 0; i < bck_mark_points->size(); ++i) {
    bck_mark_value[i] = (*bck_mark_points)[i].value;
  }
  InitGmms(*source_image, sub_mark_value, bck_mark_value, &background_gmm, &subject_gmm);

  const double gamma = 50;
  const double lambda = 9 * gamma;
  const double beta = utils::CalcBeta(*source_image);

  for(int i = 0; i < ITER_COUNT; ++i) {
    AssignGmmsComponents(*source_image, *marked_image,
                         &background_gmm, &subject_gmm, &comp_idxs);
    LearnGmms(*source_image, *marked_image, comp_idxs, &background_gmm, &subject_gmm);

  CountTime ct;
  ct.ContBegin();
    GraphCutWithGrab(*source_image, marked_image, background_gmm, subject_gmm,
                     lambda, beta, gamma, NULL);
  ct.ContEnd();
  ct.PrintTime();
  }

  CreateFinalMarkedImage(marked_image);
  utils::ExtractContourLine(m_sd);
  sd->SetCutStatus(true);
}

void GrabCut::MakeTrimapForMarkedImage(ImageData<int>* marked_image, int band_width) {
}

void GrabCut::UpdateSceneVector(SegmentationData* sd, UserInput* uip) {
}

void GrabCut::MakeGraphVtx(const ImageData<int>& marked_image) {
}

double GetEnergyRegionItem(int colour, Scene scn) {
  return 0;
}

double GetEnergyBoundryItem(int colour, int near_colour, Direction drc) {
  return 0;
}

void SegmentImageByGraph(const GraphType& graph, ImageData<int>* marked_image) {
}
