// Copyright sxniu 2014-10
#ifndef INCLUDE_LINES_H_
#define INCLUDE_LINES_H_

#include <vector>
#include <string>
#include <stack>

#include "include/UserInput.h"
#include "include/utils.h"

template <class T>
class ImageData;

class Lines :public UserInput {
 public:
  Lines();
  Lines(const char* file_name);
  Lines(Lines* half_scale_lines);
  ~Lines();
  void Reset();
  virtual void DrawFirstPointForSub(int x, int y);
  virtual void DrawFirstPointForBck(int x, int y);
  virtual void DrawSubjectBegin(int x, int y);
  virtual void DrawBackgroundBegin(int x, int y);
  virtual void DrawSubjectFinish(int x, int y);
  virtual void DrawBackgroundFinish(int x, int y);
  static int GetMarkedAreaId();
  static int GetSquareRadius();

 private:
  int ReverseLine(ImageData<int>* ui_image, ImageData<int>* marked_image,
                  int reverse_x, int reverse_y, int line_colour,
                  std::vector<UserInput::LinePoint>* line_points);
  void UpdateSquare(ImageData<int>* ui_image, ImageData<int>* scr_image,
                    ImageData<int>* marked_image,
                    int start_x, int start_y, int end_x, int end_y,
                    int reverse_number);
  void CollectMovePoints(int move_x, int move_y);
  void CollectAccessedSubjectAreaIds();
  void CheckAndRemoveAreaWhenReversing(ImageData<int>* marked_image, int index);
  double m_line_length;
  int m_line_number;
  static int m_marked_area_id;
  static double m_square_radius;
  // (index, area_id)
  std::stack<std::pair<int, int>>* m_move_points;
};

#endif  // INCLUDE_LINES_H_
