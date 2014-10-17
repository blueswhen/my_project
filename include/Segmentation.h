// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATION_H
#define INCLUDE_SEGMENTATION_H

#include <stdio.h>
#include <assert.h>

#include "include/SegmentationData.h"
#include "include/UserInput.h"

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

  Segmentation(SegmentationData* sd, UserInput* usr_input)
    : m_sd(sd)
    , m_usr_input(usr_input) {}
  virtual ~Segmentation() {}
  void SetUserInput(UserInput* usr_input) {
    if (usr_input == NULL) {
      printf("error: the usr_input is null");
      return;
    }
    assert(m_usr_input != NULL);
    m_usr_input = usr_input;
  }
  void ResetUserInput() {
    m_sd->Reset();
    m_usr_input->Reset();
  }

 protected:
  SegmentationData* m_sd;
  UserInput* m_usr_input;
};

#endif  // INCLUDE_SEGMENTATION_H
