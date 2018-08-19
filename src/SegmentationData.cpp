// Copyright sxniu 2014-10
#include "include/SegmentationData.h"
#include "include/ImageData.h"
#include "include/utils.h"

SegmentationData::SegmentationData(ImageData<int>* src_img,
                                   ImageData<int>* src_img_bck,
                                   int usr_sub_colour, int usr_bck_colour,
                                   SegmentationData* half_sd)
    : m_source_image(src_img)
    , m_source_image_backup(src_img_bck)
    , m_marked_image(ImageData<int>())
    , m_usr_sub_colour(usr_sub_colour)
    , m_usr_bck_colour(usr_bck_colour)
    , m_half_sd(half_sd)
    , m_is_cutted(false) {
      m_marked_image.CreateEmptyImage(m_source_image->GetWidth(), m_source_image->GetHeight());
      m_usr_sub_colour = m_usr_sub_colour & RIGHT_HALF;
      m_usr_bck_colour = m_usr_bck_colour & RIGHT_HALF;
      assert(m_usr_sub_colour && m_usr_bck_colour);
    }

void SegmentationData::Reset() {
  *m_source_image = *m_source_image_backup;
  ClearMarkedImage();
  m_is_cutted = false;
}

void SegmentationData::ClearMarkedImage() {
  m_marked_image.CreateEmptyImage(m_source_image->GetWidth(), m_source_image->GetHeight());
}

ImageData<int>* SegmentationData::GetSourceImage() {
  return m_source_image;
}

ImageData<int>* SegmentationData::GetSourceImageBck() {
  return m_source_image_backup;
}

SegmentationData* SegmentationData::GetHalfSegmentationData() {
  return m_half_sd;
}

ImageData<int>* SegmentationData::GetMarkedImage() {
  return &m_marked_image;
}

int SegmentationData::GetSubjectColour() {
  return m_usr_sub_colour;
}

int SegmentationData::GetBackgroundColour() {
  return m_usr_bck_colour;
}

bool SegmentationData::GetCutStatus() {
  return m_is_cutted;
}

void SegmentationData::SetCutStatus(bool status) {
  m_is_cutted = status;
}
