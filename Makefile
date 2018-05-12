#Copyright 2014-5 sxniu 
#macos

CXX = clang++
LIBS = -L/usr/local/Cellar/opencv@2/2.4.13.6_2/lib \
       -lopencv_core -lopencv_highgui -lopencv_imgproc
INCS = -I. \
       -I/usr/local/Cellar/opencv@2/2.4.13.6_2/include
CXXFLAGS = -O3 -std=c++11 -pthread -fpermissive -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.13.sdk

lazysnapping : \
  src/utils.cpp \
  src/main.cpp \
  src/CountTime.cpp \
  src/WatershedRegion.cpp \
  src/ui.cpp \
  src/Segmentation.cpp \
  src/SegmentationData.cpp \
  src/LazySnapping.cpp \
  src/Lines.cpp \
  src/Square.cpp \
  src/Lasso.cpp \
  src/GrabCut.cpp \
  src/UserInput.cpp \
  src/Gmm.cpp
	$(CXX) $(CXXFLAGS) $^ $(LIBS) $(INCS) -o $@

clean :
	rm -f lazysnapping
	rm -f *.jpg
	rm -f *.bmp
