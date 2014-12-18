// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATION_H
#define INCLUDE_SEGMENTATION_H

#include <map>
#include "include/utils.h"

class UserInput;
class SegmentationData;
class GrapyType;

template <class T>
class ImageData;

class Segmentation {
 public:
  virtual void DoPartition() = 0;
  virtual ImageData<int>* GetUiImage() = 0;
  virtual void DoLeftButtonDown(int x, int y) = 0;
  virtual void DoRightButtonDown(int x, int y) = 0;
  virtual void DoLeftMouseMove(int x, int y) = 0;
  virtual void DoRightMouseMove(int x, int y) = 0;
  virtual void DoLeftButtonUp(int x, int y) = 0;
  virtual void DoRightButtonUp(int x, int y) = 0;

  virtual void ResetUserInput();
  Segmentation(SegmentationData* sd, UserInput* usr_input);
  virtual ~Segmentation() {}
  void SetUserInput(UserInput* usr_input);
  void SegmentationWithCoarsen();
  void SegmentationWithoutCoarsen();
  static bool CheckUserMark(SegmentationData* sd, UserInput* uip);
  static void RemoveLastResult(SegmentationData* sd);

 private:
  void SegmentationForAllPixel(SegmentationData* sd, UserInput* uip);
  void UncoarsenMarkedImage(SegmentationData* sd, UserInput* uip,
                            std::map<int, int>* graph_vtx_map, int band_width);
  virtual void InitMarkedImage(SegmentationData* sd, UserInput* uip) = 0;
  virtual void Cut(SegmentationData* sd, UserInput* uip, std::map<int, int>* graph_vtx_map) = 0;
  virtual void MakeTrimapForMarkedImage(ImageData<int>* marked_image, int band_width) = 0;
  virtual void UpdateSceneVector(SegmentationData* sd, UserInput* uip) = 0;
  virtual void MakeGraphVtx(const ImageData<int>& marked_image, std::map<int, int>* graph_vtx_map) = 0;

 protected:
  SegmentationData* m_sd;
  UserInput* m_usr_input;
};

#endif  // INCLUDE_SEGMENTATION_H
