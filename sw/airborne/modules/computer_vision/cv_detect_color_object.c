/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h" //joep: I think it should be <std.h> however still does not find the file

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

#define GREEN_PIXEL_COUNTER_STOP 20 // pixels
#define GREEN_SLOPE_THRESEHOLD 20 // pixels
#define HEIGHT_FRACTION 4

// Filter Settings
uint16_t obstacleList[15]; //Tinka: The nr /3 is the max amount of detected obstacles
uint16_t obstacleListGreen[15];
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;
uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

//joep : added back old variables as they give undefined error when missing
uint8_t minHue1 = 0;
uint8_t maxHue1 = 0;
uint8_t minSat1 = 0;
uint8_t amount_of_pixels1 = 0;
bool cod_draw1 = false;
bool cod_draw2 = false;

int32_t x_c, y_c;

// define global variables 
//Tinka: TODO: check if these are okay as they are for now
struct color_object_t {
  uint16_t left_pixel;
  uint16_t right_pixel;
  uint16_t distance_measure[15]; //joep: is this size correct
  bool updated;

  //Jagga: This variable inside the global filter was used in line 148
  // uint32_t color_count; //

};
struct color_object_t global_filters[2]; // joep: this makes two intances of struct color_object_t namely globalfilters[0] and globalfilters[1]

// Function
//Tinka: the input to the function is kept the same + our 4 own variables
void find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t minHue, uint8_t maxHue,
                              uint8_t minSat, uint8_t amount_of_pixels);

                              


/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter) // joep:filter is always 1 at this point see object_detector1
{
  //Tinka: also here our variables are added and the old ones are kept for consistency
  uint8_t minHue, maxHue;
  uint8_t minSat;
  uint8_t amount_of_pixels;  

  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    //Tinka: there used to be 2 cases, but I only kept the first
    case 1:
      //Tinka: also here our variables are added and the old ones are kept for consistency
      minHue = minHue1;
      maxHue = maxHue1;
      minSat = minSat1;
      amount_of_pixels = amount_of_pixels1;

      //Tinka: old variabels
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    default:
      return img;
  };

  
  // Filter and find centroid
  //Tinka: our variables are added to the next line where we call the 'find_object_centroid' function
  find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, minHue, maxHue, minSat, amount_of_pixels);
  //Tinka: commented out the print statement because they're no longer relevant TODO: add nice new ones. USEFUL!
  //VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  //VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        //hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  //Tinka: here we add our found variables to the filter. I kept the old filter names to prevent errors, though this might be nice to change sometime
  //Tinka: also commented out old unused filter variables because I lost track of where they're send of to
  pthread_mutex_lock(&mutex);
  for(int counter = 0; counter<15; counter++){
    global_filters[filter-1].distance_measure[counter] = obstacleList[counter];
  }
  //global_filters[filter-1].x_c = left_pixel;
  //global_filters[filter-1].y_c = right_pixel;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t)); //joep: I think this creates space for the two structs
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
  #ifdef COLOR_OBJECT_DECTECTOR_LUM_MIN1
    //Tinka: we added our own variables, these are loaded from the cv_detect_color_object.xml file :)
   //Tinka: remaining variables are kept to not mess up existing 'ifdef' statements
    minHue1 = COLOR_OBJECT_DETECTOR_MINHUE;
    maxHue1 = COLOR_OBJECT_DETECTOR_MAXHUE;
    minSat1 = COLOR_OBJECT_DETECTOR_MINSAT;
    amount_of_pixels1 = COLOR_OBJECT_DETECTOR_AOP;
  
    //Tinka: said unused variables (actually I did end up using them hehe):
    cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
    cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
    cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
    cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
    cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
    cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
  #endif
  #ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
  #endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif
}
/*
 * find_object_centroid
 *
 * Tinka: the name of this function is no longer representative, but isn't changed to once again prevent the code from crashing 
 * somewhere else :) also our variables are added to the input
 * The code now calculates a list of the found objects containing their most left and right pixel 
 */
void find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t minHue, uint8_t maxHue,
                              uint8_t minSat, uint8_t amount_of_pixels) {
    //Tinka: here I added the variables that we'll be needing and I removed the useless old ones
    uint32_t orangeCount = 0;
    uint8_t rowList[520];
//    printf("rowlist printing session");
//    for (int loopyloop; loopyloop <520; loopyloop ++){
//        printf("%d", rowList[loopyloop]);
//    }
    uint8_t *buffer = img->buf;
    uint8_t foundobstacles = 0;
    int32_t left_pixel = 0;
    int32_t right_pixel = 0;
    int32_t min_nrofCols = 10; //Tinka: min amount of detected columns in a row for a positive
    uint16_t index = 0;
    uint8_t Joep[240][520];
    uint8_t Joepcolumnlist[520];
    uint8_t greenOutline[img->h/HEIGHT_FRACTION];

    for (uint8_t loop = 0; loop < 15; loop++) {
        obstacleList[loop] = 0;
        obstacleListGreen[loop] = 0;
    }
    for (uint8_t i = 0; i < img->h/HEIGHT_FRACTION; i++) {
      greenOutline[i] = 0;
    }
    //Tinka: 'y' changed to 'row', 'x' changes to 'col' for my sanity :)
    for (uint16_t row = 0; row < img->h; row++) {
        rowList[row] =0;
        Joepcolumnlist[row]=0;
        uint8_t notGreenCounter = 0;
        for (uint16_t col = img->w-1; col>=0; col++) {
            //int currentPixel = row + col * img->h
            //check if the color is inside the specified values
            uint8_t *yp, *up, *vp;

            //[u,y1,v,y2,u,y3,v,y4]
            if (col % 2 == 0) {
                // Even x
                up = &buffer[row * 2 * img->w + 2 * col];      // U
                yp = &buffer[row * 2 * img->w + 2 * col + 1];  // Y1
                vp = &buffer[row * 2 * img->w + 2 * col + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            } else {
                // Uneven x
                up = &buffer[row * 2 * img->w + 2 * col - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * col  - 1]; // Y1
                vp = &buffer[row * 2 * img->w + 2 * col];      // V
                yp = &buffer[row * 2 * img->w + 2 * col + 1];  // Y2
            }
//            if ((*yp >= lum_min) && (*yp <= lum_max) &&
//                (*up >= cb_min) && (*up <= cb_max) &&
//                (*vp >= cr_min) && (*vp <= cr_max)) {
//                orangeCount++;
//            }
//this if statement determines the color detection this one below is based on the sim dataset
            if (isOrange_yuv(yp, up, vp)){
              Joep[col][row] = 1;
              Joepcolumnlist[col]++;
            }
            else {
              Joep[col][row] = 0;
            }

            /* Green pixel detection */
            if (notGreenCounter < GREEN_PIXEL_COUNTER_STOP){
              if (row%HEIGHT_FRACTION==0) {
                if (isGreen_yuv(yp, up, vp)) {
                  notGreenCounter = 0;
                  greenOutline[row/HEIGHT_FRACTION] = row;
                }
              }

            }
          }
            for(uint16_t i=0; i<520;i++) {
                if (Joepcolumnlist[i] >= amount_of_pixels) {
                    rowList[i] = 1;
                }
            }

        }
    printf("BEGIN________________________________________________________________________________________");
//    for (int row =0; row < 240; row++){
//        //printf("begin new row %d \n",i);
//        for(int col =0; col<520; col++){
//            printf("%d,",Joep[row][col]);
//            if(Joep[row][col])
//        }
//    }
    //joep: Beun solution take middle row and equal to rowList
//        for(int col = 0; col<520; col++){
//            int middle = 120;
//            rowList[col] = Joep[middle][col];
//        }
    for (index = 0; index < 520; index++) {
        //Tinka: checking where we go from 0 to 1 value (begin of obstacle)
        if ((rowList[index] == 0) &&
            (rowList[index + 1] == 1)) {
            left_pixel = index + 1;
        }

        //Tinka: checking where we go from 1 to 0 value (end of obstacle)
        if ((rowList[index] == 1) &&
            (rowList[index + 1] == 0)) {
            right_pixel = index;
            //Tinka: checking if the obstacle is wide enough to be interesting
            if (right_pixel - left_pixel >= min_nrofCols) {
                obstacleList[foundobstacles] = left_pixel;
                obstacleList[foundobstacles + 1] = right_pixel;
                obstacleList[foundobstacles + 2] = right_pixel - left_pixel + 1;
                foundobstacles += 3;
            }


        }
        //Tinka: wow :D
        //return obstacleList;
    }
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  //Tinka: here the last value is pushed as 'quality' to the orange_avoid file. Here our data will be put :)
  if(local_filters[0].updated)
  {
//AbiSendMsgVISUAL_DETECTION(uint8_t sender_id, int16_t pixel_x, int16_t pixel_y, int16_t pixel_width, int16_t pixel_height, 
//int32_t quality1, int32_t quality2, int32_t quality3, int32_t quality4, int32_t quality5, int32_t quality6, int32_t quality7, int32_t quality8, int32_t quality9, int32_t quality10, int32_t quality11, int32_t quality12, int32_t quality13, int32_t quality14, int32_t quality15, int16_t extra) {


    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].left_pixel, local_filters[0].right_pixel,
        0, 0, 
        obstacleList[0],obstacleList[1],obstacleList[2],
        obstacleList[3],obstacleList[4],obstacleList[5],
        obstacleList[6],obstacleList[7],obstacleList[8],
        obstacleList[9],obstacleList[10],obstacleList[11],
        obstacleList[12],obstacleList[13],obstacleList[14], 0);
    local_filters[0].updated = false;
    //VERBOSE_PRINT("printing left pixel %d and the right pixel %d of the first obstacle found", obstacleList[0], obstacleList[1]);
    printf("%d,%d",obstacleList[0], obstacleList[1]);
  }
}

bool isGreen_yuv(uint8_t *yp, uint8_t *up, uint8_t *vp){
  bool is_green;
  if (*vp >= 168){ // orange values to be changed to green vlaues
    if (*vp <= 178){
        if (*yp > 70){
            is_green = true;
        }
    }
    else if(*yp >= 80){
        is_green = true;
    }
  } else{
    is_green = false;
  }
  return is_green;
}

bool isOrange_yuv(uint8_t *yp, uint8_t *up, uint8_t *vp){
  bool is_green;
  if (*vp >= 168){ 
    if (*vp <= 178){
        if (*yp > 70){
            is_green = true;
        }
    }
    else if(*yp >= 80){
        is_green = true;
    }
  } else{
    is_green = false;
  }
  return is_green;
}
