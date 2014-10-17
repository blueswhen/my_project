// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATIONDATA_H
#define INCLUDE_SEGMENTATIONDATA_H

#include <vector>

template <class T>
class ImageData;

class SegmentationData {
 public:
  SegmentationData(ImageData<int>* src_img, ImageData<int>* src_img_bck,
                   int usr_sub_colour, int usr_bck_colour);
  ~SegmentationData();
  void Reset();
  void ClearMarkedImage();
  ImageData<int>* GetSourceImage();
  ImageData<int>* GetSourceImageBck();
  ImageData<int>* GetMarkedImage();
  int GetSubjectColour();
  int GetBackgroundColour();

 private:
  ImageData<int>* m_source_image;
  ImageData<int>* m_source_image_backup;
  ImageData<int>* m_marked_image;
  int m_usr_sub_colour;
  int m_usr_bck_colour;
};

#endif  // INCLUDE_SEGMENTATIONDATA_H
