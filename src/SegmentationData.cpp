// Copyright sxniu 2014-10
#include "include/SegmentationData.h"
#include "include/ImageData.h"

SegmentationData::SegmentationData(ImageData<int>* src_img, ImageData<int>* src_img_bck)
    : m_source_image(src_img)
    , m_source_image_backup(src_img_bck)
    , m_marked_image(new ImageData<int>()) {}

SegmentationData::~SegmentationData() {
  if (m_marked_image != NULL) {
    delete m_marked_image;
    m_marked_image = NULL;
  }
}

void SegmentationData::Reset() {
  m_source_image->CopyImage(*m_source_image_backup);
  ClearMarkedImage();
}

void SegmentationData::ClearMarkedImage() {
  if (m_marked_image != NULL) {
    delete m_marked_image;
    m_marked_image = NULL;
  }
  m_marked_image = new ImageData<int>();
}

ImageData<int>* SegmentationData::GetSourceImage() {
  return m_source_image;
}

ImageData<int>* SegmentationData::GetSourceImageBck() {
  return m_source_image_backup;
}

ImageData<int>* SegmentationData::GetMarkedImage() {
  return m_marked_image;
}
