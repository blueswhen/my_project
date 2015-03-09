// Copyright sxniu 2014-10

#include "include/Segmentation.h"

#include <stdio.h>
#include <assert.h>

#include "include/SegmentationData.h"
#include "include/UserInput.h"
#include "include/ImageData.h"

#define CONTOUR_LINE RED 

Segmentation::Segmentation(SegmentationData* sd, UserInput* usr_input)
  : m_sd(sd)
  , m_usr_input(usr_input)
  , m_graph_vtx_map(NULL) {
  m_usr_input->SetSegmentationData(sd);

  SegmentationData* half_sd = m_sd->GetHalfSegmentationData();
  UserInput* half_uip = m_usr_input->GetHalfScaleUserInput();
  while (half_uip != NULL) {
    assert(half_sd != NULL);
    half_uip->SetSegmentationData(half_sd);
    half_uip = half_uip->GetHalfScaleUserInput();
    half_sd = half_sd->GetHalfSegmentationData();
  }
}

ImageData<int>* Segmentation::GetUiImage() {
  return m_sd->GetSourceImage();
}

ImageData<int>* Segmentation::GetMarkedImage() {
  return m_sd->GetMarkedImage();
}

void Segmentation::ResetUserInput() {
  m_sd->Reset();
  m_usr_input->Reset();

  SegmentationData* half_sd = m_sd->GetHalfSegmentationData();
  UserInput* half_uip = m_usr_input->GetHalfScaleUserInput();
  while (half_uip != NULL) {
    assert(half_sd != NULL);
    half_uip->Reset();
    half_sd->Reset();
    half_uip = half_uip->GetHalfScaleUserInput();
    half_sd = half_sd->GetHalfSegmentationData();
  }
}

void Segmentation::RemoveLastResult(SegmentationData* sd) {
  if (!sd->GetCutStatus()) {
    return;
  }
  ImageData<int>* source_image = sd->GetSourceImageBck();
  ImageData<int>* result_image = sd->GetSourceImage();
  int width = source_image->GetWidth();
  int height = source_image->GetHeight();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int colour = GET_PIXEL(result_image, index);
      if (colour == CONTOUR_LINE) {
        int src_col = GET_PIXEL(source_image, index);
        SET_PIXEL(result_image, index, src_col);
      }
    }
  }
}

bool Segmentation::CheckUserMark(SegmentationData* sd, UserInput* uip) {
  assert(sd != NULL && uip != NULL);
  const ImageData<int>* source_image = sd->GetSourceImageBck();
  std::vector<int>* sub_mark_index = uip->GetSubjectPoints().first;
  std::vector<int>* sub_mark_value = uip->GetSubjectPoints().second;
  std::vector<int>* bck_mark_index = uip->GetBackgroundPoints().first;
  std::vector<int>* bck_mark_value = uip->GetBackgroundPoints().second;

  if (sub_mark_index->size() == 0) {
    printf("error: no sub line mark\n");
    return false;
  }
  assert(sub_mark_value->size() == sub_mark_index->size());
  // if no bck line input, replace it with four boards of the image
  if (bck_mark_index->size() == 0) {
    int width = source_image->GetWidth();
    int height = source_image->GetHeight();
    int gap = 0;
    for (int y = gap; y < height - gap; y += height - 2 * gap) {
      for (int x = gap; x < width - gap; ++x) {
        int index = y * width + x;
        int colour = GET_PIXEL(source_image, index);
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(colour);
      }
    }
    for (int x = gap; x < width - gap; x += width - 2 * gap - 1) {
      for (int y = gap; y < height - gap; ++y) {
        int index = y * width + x;
        int colour = GET_PIXEL(source_image, index);
        bck_mark_index->push_back(index);
        bck_mark_value->push_back(colour);
      }
    }
  }
  return true;
}

void Segmentation::SetUserInput(UserInput* usr_input) {
  if (usr_input == NULL) {
    printf("error: the usr_input is null");
    return;
  }
  assert(m_usr_input != NULL);
  m_usr_input = usr_input;
  m_usr_input->SetSegmentationData(m_sd);
}

void Segmentation::SegmentationForAllPixel(SegmentationData* sd, UserInput* uip) {
  bool is_continue = CheckUserMark(sd, uip);
  if (!is_continue) {
    printf("user mark not ready!\n");
    return;
  }
  InitMarkedImage(sd, uip);
  Cut(sd, uip);
}

void Segmentation::UncoarsenMarkedImage(SegmentationData* sd, UserInput* uip, int band_width) {
  ImageData<int>* half_marked_image = sd->GetHalfSegmentationData()->GetMarkedImage();
  ImageData<int>* marked_image = sd->GetMarkedImage();
  utils::DoubleScale(*half_marked_image, marked_image);
  MakeTrimapForMarkedImage(marked_image, band_width);
  UpdateSceneVector(sd, uip);
  MakeGraphVtx(*marked_image);
}

void Segmentation::SegmentationWithCoarsen() {
  SegmentationData* sd = m_sd;
  UserInput* uip = m_usr_input;

  std::vector<SegmentationData*> sd_vec;
  std::vector<UserInput*> uip_vec;
  while (uip != NULL) {
    sd_vec.push_back(sd);
    uip_vec.push_back(uip);
    uip = uip->GetHalfScaleUserInput();
    sd = sd->GetHalfSegmentationData();
  }

  std::vector<SegmentationData*>::reverse_iterator itr_sd = sd_vec.rbegin();
  std::vector<UserInput*>::reverse_iterator itr_uip = uip_vec.rbegin();
  SegmentationForAllPixel(*itr_sd, *itr_uip);
  int band_width = 3;
  while (true) {
    // band_width *= 2;
    ++itr_uip;
    ++itr_sd;
    if (itr_uip == uip_vec.rend()) {
      break;
    }

    m_graph_vtx_map = new std::unordered_map<int, int>();
    UncoarsenMarkedImage(*itr_sd, *itr_uip, band_width);
    Cut(*itr_sd, *itr_uip);
    delete m_graph_vtx_map;
    m_graph_vtx_map = NULL;
  }
}

void Segmentation::SegmentationWithoutCoarsen() {
  SegmentationForAllPixel(m_sd, m_usr_input);
}
