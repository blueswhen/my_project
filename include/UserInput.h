// Copyright sxniu 2014-10
#ifndef INCLUDE_USER_INPUT_H_
#define INCLUDE_USER_INPUT_H_

#include <vector>
#include <stdlib.h>

class SegmentationData;

template <class T>
class ImageData;

class UserInput {
 friend class Lines;
 public:
  virtual void DrawFirstPointForSub(int x, int y) {}
  virtual void DrawSubjectBegin(int x, int y) {}
  virtual void DrawFirstPointForBck(int x, int y) {}
  virtual void DrawBackgroundBegin(int x, int y) {}
  virtual void DrawSubjectFinish(int x, int y) {}
  virtual void DrawBackgroundFinish(int x, int y) {}

  UserInput();
  UserInput(UserInput* hlf_uip);
  void Reset();
  virtual ~UserInput() {}
  void SetSegmentationData(SegmentationData* sd);
  UserInput* GetHalfScaleUserInput();
  std::pair<std::vector<int>*, std::vector<int>* > GetSubjectPoints();
  std::pair<std::vector<int>*, std::vector<int>* > GetBackgroundPoints();

 protected:
  SegmentationData* m_sd;
  std::vector<int> m_sub_mark_index;
  std::vector<int> m_sub_mark_value;
  std::vector<int> m_bck_mark_index;
  std::vector<int> m_bck_mark_value;
  UserInput* m_hlf_uip;
};

#endif  // INCLUDE_USER_INPUT_H_
