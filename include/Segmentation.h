// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATION_H
#define INCLUDE_SEGMENTATION_H

template <class T>
class ImageData;

class Segmentation {
 public:
  virtual void DoPartition() = 0;
  virtual void RemoveLastResult() = 0;
  virtual ImageData<int>* GetUiImage() = 0;
  virtual void DoLeftButtonDown(int index) = 0;
  virtual void DoRightButtonDown(int index) = 0;
  virtual void DoLeftMouseMove(int index) = 0;
  virtual void DoRightMouseMove(int index) = 0;
  virtual void DoLeftButtonUp(int index) = 0;
  virtual void DoRightButtonUp(int index) = 0;
};

#endif  // INCLUDE_SEGMENTATION_H
