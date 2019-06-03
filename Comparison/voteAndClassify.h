/*
 * Handles determining the classification of the unknown
 * image based on the K nearest neighbors
 * Author: Melanie Mendoza
 */

#ifndef VOTEANDCLASSIFY_H_INCLUDED
#define VOTEANDCLASSIFY_H_INCLUDED

/*
 * Looks at the K (as defined in comparisonArray.h) nearest
 * neighbors and assigns a label to the unknown image by voting
 * with the labels of the nearest neighbors.
 * Returns the label as a char and a measure of confidence
 * in the parameter.
 * Confidence is calculated as a percent from the number of
 * neighbors with the winning label over K.
 */
unsigned char voteAndClassify(unsigned int* confidence);


#endif // VOTEANDCLASSIFY_H_INCLUDED
