#include "medianFilter.h"

MedianFilter::MedianFilter() {
    for (int i = 0; i < 5; ++i) {
        values[i] = 0.0f;
        sorted[i] = 0.0f;
    }
    currentIndex = 0;
}

MedianFilter::~MedianFilter() {}

float MedianFilter::filter(float val) {
    values[currentIndex] = val;
    currentIndex++;
    if (currentIndex >= 5) {
        currentIndex = 0;
    }

    for (int i = 0; i < 5; i++) {
        sorted[i] = values[i];
    }

    insertionSort(sorted, 5);

    return sorted[2];
}

void MedianFilter::insertionSort(float arr[], int n) {
    int i, j;
    float key;
    for (i = 1; i < n; i++) {
        key = arr[i];
        j = i - 1;

        // Move elements of arr[0..i-1], that are greater than key, to one position ahead of their current position
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
}