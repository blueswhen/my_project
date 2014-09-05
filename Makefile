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
  include/data.h \
  include/lazy_snapping.h \
  src/utils.cpp \
  src/main.cpp \
  src/CountTime.cpp \
  src/WatershedRegion.cpp \
  src/ui.cpp \
  src/lazy_snapping.cpp
	$(CXX) $^ $(LIBS) $(INCS) -o $@ -O2

clean :
	rm -f lazysnapping
	rm -f *.jpg
	rm -f *.bmp
