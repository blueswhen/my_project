// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATION_H
#define INCLUDE_SEGMENTATION_H

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
  static bool CheckUserMark(SegmentationData* sd, UserInput* uip);
  static void RemoveLastResult(SegmentationData* sd);

 protected:
  SegmentationData* m_sd;
  UserInput* m_usr_input;
};

#endif  // INCLUDE_SEGMENTATION_H
