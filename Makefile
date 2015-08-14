#Copyright 2014-5 sxniu 

CXX = g++
LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc
INCS = -I. -std=c++11 -pthread
CXXFLAGS = -O3

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
  include/UserInput.h \
  include/Lines.h \
  include/Square.h \
  include/Lasso.h \
  include/Data.h \
  include/GrabCut.h \
  include/Gmm.h \
	include/Graph.h \
	include/PRGraph.h \
	include/IGraph.h \
	include/FGraph.h \
	include/IFGraph.h \
	include/ibfs/ibfs.h \
	include/maxflow-v3.03/graph.h \
	include/ibfs/ibfs.cpp \
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
