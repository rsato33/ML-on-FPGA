/*
 * A struct representing the data associated with a known image: its
 * pixel values and its label.
 * Author: Melanie Mendoza
 */

#ifndef IMAGESTRUCT_H_INCLUDED
#define IMAGESTRUCT_H_INCLUDED

typedef struct image {
    unsigned int distance;
    char label;
    struct image* next;
    struct image* prev;
} image;



#endif // IMAGESTRUCT_H_INCLUDED
