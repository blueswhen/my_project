// Copyright sxniu 2014-9
#ifndef INCLUDE_DATA_H
#define INCLUDE_DATA_H

#include "include/lazy_snapping.h"

namespace data {

struct RawData {
  lazy_snapping::LazySnappingData* lsd;
};

}  // namespace data
#endif  // INCLUDE_DATA_H
