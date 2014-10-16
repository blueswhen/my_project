// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATIONDATA_H
#define INCLUDE_SEGMENTATIONDATA_H

template <class T>
class ImageData;

class SegmentationData {
 public:
  SegmentationData(ImageData<int>* src_img, ImageData<int>* src_img_bck);
  ~SegmentationData();
  virtual void Reset();
  void ClearMarkedImage();
  ImageData<int>* GetSourceImage();
  ImageData<int>* GetSourceImageBck();
  ImageData<int>* GetMarkedImage();

 private:
  ImageData<int>* m_source_image;
  ImageData<int>* m_source_image_backup;
  ImageData<int>* m_marked_image;
};

#endif  // INCLUDE_SEGMENTATIONDATA_H
