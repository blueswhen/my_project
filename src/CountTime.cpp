// Copyright sxniu 2014-6-9

#include "include/CountTime.h"
#include <stdlib.h>
#include <stdio.h>

void CountTime::ContBegin() {
  gettimeofday(&time_start, NULL);
}

void CountTime::ContEnd() {
  gettimeofday(&time_end, NULL);
}

// unit is "us"
double CountTime::ContResult() {
  double time = (time_end.tv_sec - time_start.tv_sec) +
                (time_end.tv_usec - time_start.tv_usec) / 1000000.0;
  return time;
}

// unit is "ms"
void CountTime::PrintTime() {
  printf("the running time = %fms\n", ContResult() * 1000);
}
