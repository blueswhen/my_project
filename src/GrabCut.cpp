// Copyright sxniu 2014-9
#include "include/GrabCut.h"

#include <vector>
#include <utility>
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
#include "include/maxflow-v3.03/block.h"
#include "include/maxflow-v3.03/graph.h"
#include "include/Segmentation.h"
#include "include/SegmentationData.h"
#include "include/UserInput.h"

// k means
#define ITER 10

// grab cut
#define ITER_COUNT 5

using namespace cv;

enum GrabScene {
  BCK = 0,
  SUB = 1,
  PR_BCK = 2,
  PR_SUB = 3 
};

ImageData<int>* GrabCut::GetUiImage() {
  return m_sd->GetSourceImage();
}

void GrabCut::DoLeftButtonDown(int x, int y) {
  m_usr_input->DrawFirstPointForSub(x, y);
}

void GrabCut::DoRightButtonDown(int x, int y) {
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
  DoPartition();
}

void GrabCut::DoRightButtonUp(int x, int y) {
  m_usr_input->DrawBackgroundFinish(x, y);
  DoPartition();
}

void InitMask(ImageData<int>* marked_img, const std::vector<int>& subject_index,
              const std::vector<int>& background_index) {
  assert(marked_img != NULL && marked_img->IsEmpty() == false);
  int width = marked_img->GetWidth();
  int height = marked_img->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      SET_PIXEL(marked_img, index, PR_SUB);
    }
  }
  for (int i = 0; i < background_index.size(); ++i) {
    SET_PIXEL(marked_img, background_index[i], BCK);
  }
  if(subject_index.size() + background_index.size() != width * height) {
    for (int i = 0; i < subject_index.size(); ++i) {
      SET_PIXEL(marked_img, subject_index[i], SUB);
    }
  }
}

/*
  Initialize Gmm background and foreground models using kmeans algorithm.
*/
void InitGmms(const ImageData<int>& img, const std::vector<int>& subject_value,
              const std::vector<int>& background_value,
              Gmm* background_gmm, Gmm* subject_gmm) {
  assert(background_gmm != NULL && subject_gmm != NULL);
  assert(subject_value.size() != 0 && background_value.size() != 0);

  std::vector<int> subject_labels(subject_value.size());
  std::vector<int> background_labels(background_value.size());

  utils::Kmeans(subject_value, &subject_labels, NULL,
                Gmm::m_components_count, ITER, img.GetRandomSeed());
  utils::Kmeans(background_value, &background_labels, NULL,
                Gmm::m_components_count, ITER, img.GetRandomSeed());

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
void AssignGmmsComponents(const ImageData<int>& img, const ImageData<int>& marked_img,
                          Gmm* background_gmm, Gmm* subject_gmm,
                          std::vector<uchar>* comp_idxs) {
  assert(background_gmm != NULL && subject_gmm != NULL && comp_idxs != NULL);
  int width = img.GetWidth();
  int height = img.GetHeight();
  assert(comp_idxs->size() == width * height);
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      int index = y * width + x;
      int color = GET_PIXEL(&img, index);
      int mask_col = GET_PIXEL(&marked_img, index);
      (*comp_idxs)[index] = mask_col == BCK || mask_col == PR_BCK ?
                            background_gmm->WhichComponent(color) :
                            subject_gmm->WhichComponent(color);
    }
  }
}

/*
  Learn Gmms parameters.
*/
void LearnGmms(const ImageData<int>& img, const ImageData<int>& marked_img,
               const std::vector<uchar>& comp_idxs,
               Gmm* background_gmm, Gmm* subject_gmm) {
  background_gmm->InitLearning();
  subject_gmm->InitLearning();
  int width = img.GetWidth();
  int height = img.GetHeight();
  assert(comp_idxs.size() == width * height);
  for(int ci = 0; ci < Gmm::m_components_count; ++ci) {
    for(int y = 0; y < height; ++y) {
      for(int x = 0; x < width; ++x) {
        int index = y * width + x;
        int mask_col = GET_PIXEL(&marked_img, index);
        int colour = GET_PIXEL(&img, index);
        if(comp_idxs[index] == ci) {
          if(mask_col == BCK || mask_col == PR_BCK) {
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

void GraphCutWithGrab(const ImageData<int>& img, ImageData<int>* marked_img,
                      const Gmm& background_gmm, const Gmm& subject_gmm,
                      double lambda, double beta, double gamma) {
  int width = img.GetWidth();
  int height = img.GetHeight();
  int vtx_count = width * height;
  int edge_count = 2 * (4 * vtx_count - 3 * (width + height) + 2);
  GraphType graph(vtx_count, edge_count);

  const double gammaDivSqrt2 = gamma / std::sqrt(2.0f);
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      int index = y * width + x;
      int color = GET_PIXEL(&img, index);
      int mask_col = GET_PIXEL(marked_img, index);
      // add node
      // e1[0] is background, e1[1] is subject
      double e1[2] = {0, 0};
      if (mask_col == PR_SUB || mask_col == PR_BCK) {
        e1[0] = -log(background_gmm.SceneProbability(color));
        e1[1] = -log(subject_gmm.SceneProbability(color));
      } else if (mask_col == BCK) {
        e1[0] = 0;
        e1[1] = lambda;
      } else {
        e1[0] = lambda;
        e1[1] = 0;
      }
      int vtx_idx = graph.add_node();
      graph.add_tweights(vtx_idx, e1[0], e1[1]);

      // 8 neighbours
      if (x > 0) {
        int color_arr = GET_PIXEL(&img, y * width + x - 1);
        double e2 = gamma * exp(-beta * COLOUR_DIST_SQUARE(color, color_arr));
        graph.add_edge(vtx_idx, vtx_idx - 1, e2, e2);
      }
      if (x > 0 && y > 0) {
        int color_arr = GET_PIXEL(&img, (y - 1) * width + x - 1);
        double e2 = gammaDivSqrt2 * exp(-beta * COLOUR_DIST_SQUARE(color, color_arr));
        graph.add_edge(vtx_idx, vtx_idx - width - 1, e2, e2);
      }
      if (y > 0) {
        int color_arr = GET_PIXEL(&img, (y - 1) * width + x);
        double e2 = gamma * exp(-beta * COLOUR_DIST_SQUARE(color, color_arr));
        graph.add_edge(vtx_idx, vtx_idx - width, e2, e2);
      }
      if (x < width - 1 && y > 0) {
        int color_arr = GET_PIXEL(&img, (y - 1) * width + x + 1);
        double e2 = gammaDivSqrt2 * exp(-beta * COLOUR_DIST_SQUARE(color, color_arr));
        graph.add_edge(vtx_idx, vtx_idx - width + 1, e2, e2);
      }
    }
  }
  graph.maxflow();

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int mask_col = GET_PIXEL(marked_img, index);
      if (mask_col == PR_SUB || mask_col == PR_BCK) {
        if (graph.what_segment(index) == GraphType::SOURCE) {
          SET_PIXEL(marked_img, index, PR_SUB);
        } else {
          SET_PIXEL(marked_img, index, PR_BCK);
        }
      }
    }
  }
}

void CreateFinalMarkedImage(ImageData<int>* marked_image) {
  int width = marked_image->GetWidth();
  int height = marked_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int value = GET_PIXEL(marked_image, index);
      if (value & 1 == 1) {
        SET_PIXEL(marked_image, index, WHITE);
      } else {
        SET_PIXEL(marked_image, index, BLACK);
      }
    }
  }
}

void GrabCutPartition(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  Segmentation::RemoveLastResult(sd);
  // get user input data, these code needn't to change
  // whatever the situation is lines or square or lasso
  bool is_continue = Segmentation::CheckUserMark(sd, uip);
  if (!is_continue) {
    printf("user mark not ready!\n");
    return;
  }
  int subject_colour = sd->GetSubjectColour();
  int background_colour = sd->GetBackgroundColour();
  ImageData<int>* ui_image = sd->GetSourceImage();
  ImageData<int>* marked_image = sd->GetMarkedImage();
  const ImageData<int>* source_image = sd->GetSourceImageBck();
  std::vector<int>* sub_mark_index = uip->GetSubjectPoints().first;
  std::vector<int>* sub_mark_value = uip->GetSubjectPoints().second;
  std::vector<int>* bck_mark_index = uip->GetBackgroundPoints().first;
  std::vector<int>* bck_mark_value = uip->GetBackgroundPoints().second;

  assert(ui_image != NULL && source_image != NULL && marked_image != NULL &&
         ui_image->IsEmpty() == false && source_image->IsEmpty() == false &&
         marked_image->IsEmpty() == false);

  Gmm background_gmm;
  Gmm subject_gmm;
  std::vector<uchar> comp_idxs(source_image->GetWidth() * source_image->GetHeight());

  InitMask(marked_image, *sub_mark_index, *bck_mark_index);
  InitGmms(*source_image, *sub_mark_value, *bck_mark_value, &background_gmm, &subject_gmm);

  const double gamma = 50;
  const double lambda = 9 * gamma;
  const double beta = utils::CalcBeta(*source_image);

  for(int i = 0; i < ITER_COUNT; ++i) {
    AssignGmmsComponents(*source_image, *marked_image,
                         &background_gmm, &subject_gmm, &comp_idxs);
    LearnGmms(*source_image, *marked_image, comp_idxs, &background_gmm, &subject_gmm);
    GraphCutWithGrab(*source_image, marked_image, background_gmm, subject_gmm,
                     lambda, beta, gamma);
  }

  CreateFinalMarkedImage(marked_image);
  utils::ExtractContourLine(ui_image, marked_image);
  sd->SetCutStatus(true);
}

void GrabCut::DoPartition() {
  CountTime ct;
  ct.ContBegin();

  SegmentationData* sd = m_sd;
  UserInput* uip = m_usr_input;
  GrabCutPartition(sd, uip);

  ct.ContEnd();
  ct.ContResult();
}

double GetEnergyRegionItem(int colour, Scene scn) {
}

double GetEnergyBoundryItem(int colour, int near_colour, Direction drc) {
}

void SegmentImageByGraph(const GrapyType& graph, ImageData<int>* marked_image) {
}

