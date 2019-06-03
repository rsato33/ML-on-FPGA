/*
 * Keeps track of the K nearest neighbors and their corresponding
 * labels. Holds the macro to control the number K of nearest
 * neighbors to look at.
 * Author: Melanie Mendoza
 */

#ifndef COMPARISONARRAY_H_INCLUDED
#define COMPARISONARRAY_H_INCLUDED

#include "imageStruct.h"

// UPDATE THIS VALUE TO CHANGE THE NUMBER K OF
// NEAREST NEIGHBORS IS USED TO CLASSIFY AN IMAGE
#define K   5

/*
 * Prepares the array for use.
 */
void initializeArray();

/*
 * Inserts the image with the given distance and label if it
 * is one of the K smallest neighbors seen so far.
 */
void insertData(unsigned int distance, unsigned char label);

/*
 * Returns the distance and label data of the K nearest
 * images in the parameters. Assumes distances and labels
 * are of proper size.
 */
void getData(unsigned int* distances, unsigned char* labels);

/*
 * Returns the current number of images being stored.
 */
unsigned int arraySize();


#endif // COMPARISONARRAY_H_INCLUDED
