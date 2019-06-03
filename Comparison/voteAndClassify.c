// Author: Melanie Mendoza

#include "comparisonArray.h"

unsigned char voteAndClassify(unsigned int* confidence) {
    unsigned int minD[K];
    unsigned char minL[K];

    // Get the min K images
    getData(minD, minL);

    // Each index corresponds to the number
    // of nearest neighbors with that classification,
    // so index 0 holds the number of nearest neighbors
    // with a label of '0', and so on.
    unsigned int classificationBuckets[10];

    // Reset and populate the bucket array with counts
    for (int i = 0; i < 10; i++) {
        classificationBuckets[i] = 0;
    }
    for (int q = 0; q < K; q++) {
        classificationBuckets[(int) (minL[q] - '0')] += 1;
    }

    // Looks for the bucket with the most number of
    // neighbors. If there are equal numbers, the earliest
    // encountered one is taken. If they are all equal, the
    // label of the closest neighbors is taken.
    unsigned char minBucket = minL[0];
    unsigned int minBucketCount = classificationBuckets[(int) (minL[0] - '0')];

    for (int i = 1; i < K; i++) {
        // Grab the count for the next smallest distance
        unsigned int currCount = classificationBuckets[(int) (minL[i] - '0')];

        // Update the label if the neighbor count is greater for that label
        if (currCount > minBucketCount) {
            minBucket = minL[i];
            minBucketCount = currCount;
        }
    }

    // Confidence is number of neighbors with the given label
    // over the number K of neighbors looked at.
    *confidence = (int) ((minBucketCount * 100.0) / K);

    return minBucket;
}
