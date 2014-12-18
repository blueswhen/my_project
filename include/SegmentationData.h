// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATIONDATA_H
#define INCLUDE_SEGMENTATIONDATA_H

#include <vector>
#include "include/ImageData.h"

class SegmentationData {
 public:
  SegmentationData(ImageData<int>* src_img,
                   ImageData<int>* src_img_bck,
                   int usr_sub_colour, int usr_bck_colour,
                   SegmentationData* half_sd);
  void Reset();
  void ClearMarkedImage();
  ImageData<int>* GetSourceImage();
  ImageData<int>* GetSourceImageBck();
  ImageData<int>* GetMarkedImage();
  SegmentationData* GetHalfSegmentationData();
  int GetSubjectColour();
  int GetBackgroundColour();
  bool GetCutStatus();
  void SetCutStatus(bool status);

 private:
  ImageData<int>* m_source_image;
  ImageData<int>* m_source_image_backup;
  ImageData<int> m_marked_image;
  int m_usr_sub_colour;
  int m_usr_bck_colour;
  SegmentationData* m_half_sd;
  bool m_is_cutted;
};

#endif  // INCLUDE_SEGMENTATIONDATA_H
