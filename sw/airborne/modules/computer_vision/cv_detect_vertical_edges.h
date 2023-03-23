//
// Created by joep on 21-3-23.
//

/**
 * @file modules/computer_vision/cv_detect_vertical_edges.h
 * uses Sobel to detect vertical edges in a region (bottom/top) of the image
 */

#ifndef DRONE_STRIJDERS_CV_DETECT_VERTICAL_EDGES_H
#define DRONE_STRIJDERS_CV_DETECT_VERTICAL_EDGES_H


#include <stdint.h>
#include <stdbool.h>

// Module settings
extern uint8_t cod_lum_min1;
extern uint8_t cod_lum_max1;
extern uint8_t cod_cb_min1;
extern uint8_t cod_cb_max1;
extern uint8_t cod_cr_min1;
extern uint8_t cod_cr_max1;

extern uint8_t cod_lum_min2;
extern uint8_t cod_lum_max2;
extern uint8_t cod_cb_min2;
extern uint8_t cod_cb_max2;
extern uint8_t cod_cr_min2;
extern uint8_t cod_cr_max2;

extern bool cod_draw1;
extern bool cod_draw2;

// Module functions changed names
extern void vertical_edge_detector_init(void);
extern void vertical_edge_detector_periodic(void);

#endif //DRONE_STRIJDERS_CV_DETECT_VERTICAL_EDGES_H

