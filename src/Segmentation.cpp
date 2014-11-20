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
  , m_usr_input(usr_input) {
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
