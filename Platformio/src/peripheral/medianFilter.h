#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class MedianFilter {
public:
    MedianFilter();
    ~MedianFilter();
    
    float filter(float val);
    
private:
    float values[5];
    float sorted[5];
    int currentIndex;
    void insertionSort(float arr[], int n);
};
