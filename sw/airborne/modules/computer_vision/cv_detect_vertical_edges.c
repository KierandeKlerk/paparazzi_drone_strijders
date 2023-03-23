//
// Created by joep on 21-3-23.
//

#include "cv_detect_vertical_edges.h"

/**
 * @file modules/computer_vision/cv_detect_vertical_edges.h
 * uses Sobel to detect vertical edges in a region (bottom/top) of the image
 */

//copied from cv_detect_color_objects.c from tinka branch and altered first file name
#include "modules/computer_vision/cv_detect_vertical_edges.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include <std.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif
