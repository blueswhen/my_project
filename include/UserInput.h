// Copyright sxniu 2014-10
#ifndef INCLUDE_USER_INPUT_H_
#define INCLUDE_USER_INPUT_H_

#include <vector>
#include <stdlib.h>
#include <string>
#include "include/utils.h"

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
  virtual void Reset();
  struct LinePoint {
    int index;
    int value;
    int direction_x;
    int direction_y;
    LinePoint()
      : index(0)
      , value(0)
      , direction_x(0)
      , direction_y(0) {}
  };

  UserInput();
  UserInput(const char* file_name);
  UserInput(UserInput* hlf_uip);
  virtual ~UserInput() {}
  void SetSegmentationData(SegmentationData* sd);
  UserInput* GetHalfScaleUserInput();
  std::vector<LinePoint>* GetSubjectPoints();
  std::vector<LinePoint>* GetBackgroundPoints();
  std::string GetImageName();
  Scene GetUsrInputScene();
  void SetUsrInputScene(Scene scn);
  bool IsCut();
  void SetIsCut(bool is_cut);

 protected:
  SegmentationData* m_sd;
  std::vector<LinePoint> m_sub_line_points;
  std::vector<LinePoint> m_bck_line_points;
  UserInput* m_hlf_uip;
  std::string m_file_name;
  Scene m_usr_input_scene;

 private:
  bool m_is_cut;
};

#endif  // INCLUDE_USER_INPUT_H_
