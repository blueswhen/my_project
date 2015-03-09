// Copyright sxniu 2014-9
#ifndef  INCLUDE_GRAB_CUT_H_
#define  INCLUDE_GRAB_CUT_H_

#include <map>

#include "include/Segmentation.h"
#include "include/utils.h"

class SegmentationData;
class UserInput;
class GrapyType;

template <class T>
class ImageData;

class GrabCut :public Segmentation {
 public:
  GrabCut(SegmentationData* sd, UserInput* usr_input)
    : Segmentation(sd, usr_input) {}
  virtual void DoPartition();
  virtual void DoLeftButtonDown(int x, int y);
  virtual void DoRightButtonDown(int x, int y);
  virtual void DoLeftMouseMove(int x, int y);
  virtual void DoRightMouseMove(int x, int y);
  virtual void DoLeftButtonUp(int x, int y);
  virtual void DoRightButtonUp(int x, int y);

 private:
  virtual void InitMarkedImage(SegmentationData* sd, UserInput* uip);
  virtual void Cut(SegmentationData* sd, UserInput* uip);
  virtual void MakeTrimapForMarkedImage(ImageData<int>* marked_image, int band_width);
  virtual void UpdateSceneVector(SegmentationData* sd, UserInput* uip);
  virtual void MakeGraphVtx(const ImageData<int>& marked_image);
};

#endif  // INCLUDE_GRAB_CUT_H_
