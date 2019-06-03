/* strtok example */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void printImage(unsigned char image[784], int size) {
    printf("Printing: \n");
    for(int i = 0; i < size; i++) {
        int value = (int)image[i];
        printf("%d ", value);
    }
    printf("\n");
}

int main() {
    FILE* fp = fopen("/Users/lhc/Documents/School/EE 371/finalProjectTests/imageData.txt", "r");
    char* line;
    size_t lineSize;
    ssize_t lineLength;
    unsigned char image[784];
    int pixelPos = 0;
    int imageCount = 0;
    while((lineLength = getline(&line, &lineSize, fp)) != -1) {
        char *pch;
        pch = strtok(line, " ");
        image[pixelPos++] = (unsigned char)atoi(pch);
        while (pch != NULL && pixelPos < 784) {
            pch = strtok(NULL, " ");
            if(pch != NULL) {
                image[pixelPos++] = (unsigned char) atoi(pch);
            }
        }
        if(pixelPos != 784) {
            return 21;
        }
        printImage(image, 784);
        pixelPos = 0;
        imageCount++;
    }
    printf("Number of images: %d", imageCount);
    return 0;
}