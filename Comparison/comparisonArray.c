// Author: Melanie Mendoza

#include "comparisonArray.h"
#include "imageStruct.h"

image list[K];

unsigned int size = 0;

void initializeArray() {
    for (int q = 0; q < size; q++) {
        list[q].distance = 0;
        list[q].label = '0';
    }
    size = 0;
}


void insertData(unsigned int distance, unsigned char label) {
    if (size == 0) {
        list[0].distance = distance;
        list[0].label = label;
        size = size + 1;
        return;
    }

    unsigned int index = 0;

    while (index < size && list[index].distance <= distance) {
        index = index + 1;
    }

    // distance is the largest, only insert if size < K
    if (index == size) {
        if (size < K) {
            list[size].distance = distance;
            list[size].label = label;
            size = size + 1;
        }

    // distance isn't the largest, and index holds the index of the value
    // that should be after the parameter values in list, so insert it in the list
    // and shift distances up to size K if needed
    } else {
        unsigned int currDistance = distance;
        unsigned char currLabel = label;

        unsigned int tempDistance = list[index].distance;
        unsigned char tempLabel = list[index].label;

        // Replace value at index and increment index
        list[index].distance = currDistance;
        list[index].label = currLabel;
        index = index + 1;

        // Bubble until size or size - 1 if size == K
        while (index < size) {
            // copy the current values
            currDistance = list[index].distance;
            currLabel = list[index].label;

            // put the temp values here
            list[index].distance = tempDistance;
            list[index].label = tempLabel;

            // set the current values to the temp values
            tempDistance = currDistance;
            tempLabel = currLabel;

            // increment distance
            index = index + 1;
        }

        // If there's still space in the list, increment
        // size and set the stored temp values there
        if (size < K) {
            list[index].distance = tempDistance;
            list[index].label = tempLabel;
            size = size + 1;
        }

    }

}

void getData(unsigned int* distances, unsigned char* labels) {
    for (int i = 0; i < size; i++) {
        distances[i] = list[i].distance;
        labels[i] = list[i].label;
    }
}

unsigned int arraySize() {
    return size;
}
