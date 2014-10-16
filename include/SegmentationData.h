// Copyright sxniu 2014-10
#ifndef INCLUDE_SEGMENTATIONDATA_H
#define INCLUDE_SEGMENTATIONDATA_H

#include <vector>

template <class T>
class ImageData;

class SegmentationData {
 public:
  enum UserInputType {
    LINES,
    SQUARE,
    LASSO
  };
  
  struct LinesData {
    int m_sub_line_colour;
    int m_bck_line_colour;
    std::vector<int> m_sub_mark_index;
    std::vector<int> m_bck_mark_index;
    LinesData(int sub_line_colour, int bck_line_colour)
      : m_sub_line_colour(sub_line_colour)
      , m_bck_line_colour(bck_line_colour) {}
    void Reset() {
      m_sub_mark_index.clear();
      m_bck_mark_index.clear();
    }
  };

  struct SquareData {
    int m_square_colour;
    int m_left_up_point;
    int m_right_down_point;
    SquareData(int square_colour)
      : m_square_colour(square_colour)
      , m_left_up_point(0)
      , m_right_down_point(0) {}
    void Reset() {
      m_left_up_point = 0;
      m_right_down_point = 0;
    }
  };

  struct LassoData {
    int m_line_colour;
    std::vector<int> m_line_mark_index;
    LassoData(int line_colour)
      : m_line_colour(line_colour) {}
    void Reset() {
      m_line_mark_index.clear();
    }
  };

  SegmentationData(ImageData<int>* src_img, ImageData<int>* src_img_bck);
  ~SegmentationData();
  virtual void Reset();
  void ClearMarkedImage();
  ImageData<int>* GetSourceImage();
  ImageData<int>* GetSourceImageBck();
  ImageData<int>* GetMarkedImage();
  void SetUserInputType(UserInputType uit);
  UserInputType GetUserInputType();

 private:
  ImageData<int>* m_source_image;
  ImageData<int>* m_source_image_backup;
  ImageData<int>* m_marked_image;
  UserInputType m_uit;
};

#endif  // INCLUDE_SEGMENTATIONDATA_H
