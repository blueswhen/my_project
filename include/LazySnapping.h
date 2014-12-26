// Copyright sxniu 2014-9
#ifndef  INCLUDE_LAZY_SNAPPING_H_
#define  INCLUDE_LAZY_SNAPPING_H_

#include <vector>
#include <map>

#include "include/Segmentation.h"
#include "include/utils.h"

class SegmentationData;
class UserInput;
class WatershedRegionGroup;

template <class T>
class ImageData;

class LazySnapping :public Segmentation {
 public:
  enum LazySnappingType {
    WATERSHED,
    PIXEL
  };
  LazySnapping(SegmentationData* sd, UserInput* usr_input);

  virtual void DoPartition();
  virtual ImageData<int>* GetUiImage();
  virtual void DoLeftButtonDown(int x, int y);
  virtual void DoRightButtonDown(int x, int y);
  virtual void DoLeftMouseMove(int x, int y);
  virtual void DoRightMouseMove(int x, int y);
  virtual void DoLeftButtonUp(int x, int y);
  virtual void DoRightButtonUp(int x, int y);
  virtual void ResetUserInput();
  // double GetEnergyRegionItem(int colour, Scene scn);
  // double GetEnergyBoundryItem(int colour, int near_colour, Direction drc);
  // void SegmentImageByGraph(const GrapyType& graph, ImageData<int>* marked_image);

  void SetLazySnappingMethod(LazySnappingType lst);
  void LazySnappingForAllPixel(SegmentationData* sd, UserInput* uip);

 private:
  void InitMarkedImage();
  virtual void InitMarkedImage(SegmentationData* sd, UserInput* uip);
  virtual void Cut(SegmentationData* sd, UserInput* uip, std::map<int, int>* graph_vtx_map);
  virtual void MakeTrimapForMarkedImage(ImageData<int>* marked_image, int band_width);
  virtual void UpdateSceneVector(SegmentationData* sd, UserInput* uip);
  virtual void MakeGraphVtx(const ImageData<int>& marked_image, std::map<int, int>* graph_vtx_map);

  LazySnappingType m_lazy_type;
};

#endif  // INCLUDE_LAZY_SNAPPING_H_
