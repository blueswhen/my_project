#Copyright 2014-5 sxniu 

CXX = g++
LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc
INCS = -I.

lazysnapping : \
  include/utils.h \
  include/ImageData.h \
  include/colour.h \
  include/CountTime.h \
  include/WatershedRegion.h \
  include/ui.h \
  include/Segmentation.h \
  include/SegmentationData.h \
  include/LazySnapping.h \
  src/utils.cpp \
  src/main.cpp \
  src/CountTime.cpp \
  src/WatershedRegion.cpp \
  src/ui.cpp \
  src/SegmentationData.cpp \
  src/LazySnapping.cpp
	$(CXX) $^ $(LIBS) $(INCS) -o $@ -O2

clean :
	rm -f lazysnapping
	rm -f *.jpg
	rm -f *.bmp
