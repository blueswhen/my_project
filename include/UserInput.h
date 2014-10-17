// Copyright sxniu 2014-10
#ifndef INCLUDE_USER_INPUT_H_
#define INCLUDE_USER_INPUT_H_

class SegmentationData;

template <class T>
class ImageData;

class UserInput {
 public:
  virtual void Reset() = 0;
  virtual void DrawFirstPointForSub(ImageData<int>* image, int pos, int sub_colour) = 0;
  virtual void DrawFirstPointForBck(ImageData<int>* image, int pos, int bck_colour) = 0;
  virtual void DrawSubjectBegin(ImageData<int>* image, int pos, int sub_colour) = 0;
  virtual void DrawBackgroundBegin(ImageData<int>* image, int pos, int bck_colour) = 0;
  virtual void DrawSubjectFinish() = 0;
  virtual void DrawBackgroundFinish() = 0;
  virtual ~UserInput() {}
};

#endif  // INCLUDE_USER_INPUT_H_
