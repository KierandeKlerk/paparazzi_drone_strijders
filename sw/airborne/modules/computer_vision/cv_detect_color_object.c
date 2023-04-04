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

//IMPORT LIBRARIES
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

//DEFINE VERBOSE PRINT FOR PRINT STATEMENTS (WE JUST USE PRINTF(""))
#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


static pthread_mutex_t mutex;

//SET CAMERA FPS TO 0 TO GET ALL NEW FRAMES
#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif






//ENDGOAL: FILL THIS LIST AS FOLLOWS: {left,right,width,left,right,width,...,width} FOR ALL OBSTACLES
uint16_t obstacleList[15];

// Hyper parameters for the green gap detector
uint8_t greenPixelCounterStop = 15; // pixels
uint8_t greenSlopeThreshold =  15; // pixels
uint8_t heightFraction = 4;

bool is_simulation = true; // using color detection for simulation or real cyberzoo
bool use_orange = true;

//FILTER SETTINGS
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

//SOME OF OUR OWN ADDED VARIABLES, WE ENDED UP NOT USING THESE :(
uint8_t minHue1 = 0;
uint8_t maxHue1 = 0;
uint8_t minSat1 = 0;
uint8_t amount_of_pixels1;
bool cod_draw1 = false;
bool cod_draw2 = false;
int32_t x_c, y_c;


//MAKE STRUCT. FUNCTION: BOOL IS STILL USED TO DETERMINE WHEN TO LOAD NEXT CAMERA FRAME :)
struct color_object_t {
  uint16_t left_pixel;
  uint16_t right_pixel;
  uint16_t distance_measure[15];
  bool updated;

};
struct color_object_t global_filters[2]; // joep: this makes two intances of struct color_object_t namely globalfilters[0] and globalfilters[1]


//INITIALISE THE MAIN FUNCTION THAT BUILDS OUR obstaclelist[15]
void find_object_centroid(struct image_t *img);

                              


static struct image_t *object_detector(struct image_t *img, uint8_t filter) // joep:filter is always 1 at this point see object_detector1
{

  //BUILD obstaclelist
  find_object_centroid(img);

  //SET THE BOOL TO TRUE
  pthread_mutex_lock(&mutex);
  for(int counter = 0; counter<15; counter++){
    global_filters[filter-1].distance_measure[counter] = obstacleList[counter];
  }
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

//GET IMG
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

//INIT FUNCTION
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
void find_object_centroid(struct image_t *img) {
  uint8_t *buffer = img->buf;         //IMAGE BUFFER
  uint8_t foundobstacles = 0;         //NUMBER OF FOUND OBSTACLES
  int32_t left_pixel = 0;             //LEFTMOST PIXEL OF OBSTACLE
  int32_t right_pixel = 0;            //RIGHTMOST PIXEL OF OBSTACLE
  int32_t min_nrofCols = 10;          //MIN AMOUNT OF POSITIVE COLUMNS IN A ROW TO DETECT IT AS AN OBSTACLE
  uint8_t orangeincolumncounter[520]; //ARRAY CONTAINING THE AMOUNT OF ORANGE PIXELS DETECTED IN A ROW
  uint8_t obstacleincolumn[520];      //ARRAY CONTAINING A BINARY FOR EVERY ROW. 1 -> POLE DETECTED. 0 -> NO POLE DETECTED
  uint8_t greenOutline[img->h/heightFraction];

  //THIS LOOP EMPTIES THE OBSTACLE LIST
  for (uint8_t loop = 0; loop < 15; loop++) {
      obstacleList[loop] = 0;
  }
  for (uint8_t i = 0; i < img->h/heightFraction; i++) {
    greenOutline[i] = 0;
  }


  if (use_orange){
    // If we chose to use orange avoider, the following code runs

    //HERE WE LOOP THROUGH ALL 520 ROWS
    for (uint16_t row = 0; row < img->h; row++) {
      //ORANGE IN COLUMN COUNTER IS SET TO 0 FOR THE NEXT COLUMN
      orangeincolumncounter[row] = 0;

      for (uint16_t col = 0; col<img->w; col++) {
          
         //NOW WE LOOP THROUGH ALL THE PIXELS AND CHECK IF THEY ARE ORANGE
        uint8_t *yp, *up, *vp;

        //OBTAINING THE CORRECT COLOR VALUES FOR THE CURRENT PIXEL
        if (col % 2 == 0) {
          // EVEN COL
          up = &buffer[row * 2 * img->w + 2 * col];      // U
          yp = &buffer[row * 2 * img->w + 2 * col + 1];  // Y1
          vp = &buffer[row * 2 * img->w + 2 * col + 2];  // V
        } else {
          // UNEVEN COL
          up = &buffer[row * 2 * img->w + 2 * col - 2];  // U
          vp = &buffer[row * 2 * img->w + 2 * col];      // V
          yp = &buffer[row * 2 * img->w + 2 * col + 1];  // Y2
        }
        //CHECKING IF THE COLOR VALUES OF THE CURRENT PIXEL ARE ORANGE
        if (isOrange_yuv(yp, up, vp, is_simulation)){
          //ORANGE! -> + 1 ON THE ORANGEINCOLUMNCOUNTER
          orangeincolumncounter[row] += 1;
        }
      }
    }
    // End of orange detector loop

  } else {
    // We chose to run the green gap detector, the following code runs

    //HERE WE LOOP THROUGH ALL 520 ROWS
    for (uint16_t row = 0; row < img->h; row++) {
      uint8_t notGreenCounter = 0;

      bool checkGreen = false;
      if (row%heightFraction==0) {
        checkGreen = true;
        // Only execute check for every 4th pixel in width
      }
      for (int32_t col = img->w -1; col>=0; col--) { // Loop over the pixels from the bottom to the top of the image 
          
        //NOW WE LOOP THROUGH ALL THE PIXELS AND CHECK IF THEY ARE ORANGE
        uint8_t *yp, *up, *vp;

        //OBTAINING THE CORRECT COLOR VALUES FOR THE CURRENT PIXEL
        if (col % 2 == 0) {
          // EVEN COL
          up = &buffer[row * 2 * img->w + 2 * col];      // U
          yp = &buffer[row * 2 * img->w + 2 * col + 1];  // Y1
          vp = &buffer[row * 2 * img->w + 2 * col + 2];  // V
        } else {
          // UNEVEN COL
          up = &buffer[row * 2 * img->w + 2 * col - 2];  // U
          vp = &buffer[row * 2 * img->w + 2 * col];      // V
          yp = &buffer[row * 2 * img->w + 2 * col + 1];  // Y2
        }
        

        //CHECKING IF THE COLOR VALUES OF THE CURRENT PIXEL ARE GREEN
        if (checkGreen){  
          if (notGreenCounter < greenPixelCounterStop){
            if (isGreen_yuv(yp, up, vp, is_simulation)) {
              notGreenCounter = 0; // Reset counter to 0
              greenOutline[row/heightFraction] = col; // Store pixel coordinate
            } else {
              notGreenCounter++; // If the pixel is not green, increment counter, when the counter has reached the threshold, the outline is considere to be found in this column of the image
            }
          }
        }
      }
    }
    // End of green gap detector loop
  }
  //WE HAVE NOW LOOPED ALL THE PIXELS, TIME TO USE THE FOUND VALUES

 
  if(!use_orange) {
    // Green gap detection logic to run when not using orange detector

    //Detect dips in green pixel outline. One potential problem arises when there are more than 5 obstacles visible and it doesn't really work
    int8_t prevDirection = 0;
    bool is_first = true;
    uint8_t obstacle_list_ind = 0;

    // Loop over all entries in greenOutline except the last one
    for (uint8_t outline_ind = 0; outline_ind < img->h/heightFraction-1; outline_ind++){
      
      uint8_t difference = greenOutline[outline_ind+1] - greenOutline[outline_ind]; // Calculate the difference between this and next entry

      if(obstacle_list_ind<15){ // Only continue if there is still space in the obstacleList (mainly as failsafe)

        if ((difference>greenSlopeThreshold) && is_first){ // If the first big difference is going upwards (indicating an obstacle that stretches out beyond the frame)
          is_first = false;
          // Store leftmost pixel, rightmost pixel and width of obstacle
          obstacleList[obstacle_list_ind] = 0;
          obstacleList[obstacle_list_ind+1] = outline_ind*heightFraction;
          obstacleList[obstacle_list_ind+2] = outline_ind*heightFraction;
          
          prevDirection = 1; // Variable keeping track of the previous slope direction

          // Increment the obstacle list index for the next obstacle
          obstacle_list_ind += 3;

        } else if ((difference< -greenSlopeThreshold) && (prevDirection!=-1)){ // If the slope goes downwards fast enough, it is considered to be the left side of an obstacle
          is_first  = false;

          // Store the leftmost pixel of the obstacle and the direction of the slope
          obstacleList[obstacle_list_ind] = outline_ind*heightFraction;
          prevDirection = -1;

        } else if ((difference>greenSlopeThreshold) && (prevDirection==-1)){  // If the slope goes back up after having gone done, it is considered to be the right side of an obstacle
          // Store rightmost pixel and obstacle width + slope direction
          obstacleList[obstacle_list_ind+1] = outline_ind*heightFraction;
          obstacleList[obstacle_list_ind+2] = obstacleList[obstacle_list_ind+1] - obstacleList[obstacle_list_ind];
          prevDirection = 1;

          // Increment obstacleList index
          obstacle_list_ind += 3; 	

        } else if ((difference>greenSlopeThreshold) && (prevDirection==1)){ // If the slope is found to go up further after having gone up previously
          // Replace rightmost pixel and obstacle width 
          obstacleList[obstacle_list_ind-2] = outline_ind*heightFraction;
          obstacleList[obstacle_list_ind-1] = obstacleList[obstacle_list_ind-2] - obstacleList[obstacle_list_ind-3];
        }
      
      }
    }

    if((prevDirection==-1) && (obstacle_list_ind<15)) { // If the last slope went downwards beyond the threshold, it is assumed there is an obstacle extending beyond the right side of the frame
      // Replace rightmost pixel and obstacle width
      obstacleList[obstacle_list_ind+1] = img->h;
      obstacleList[obstacle_list_ind+2] = obstacleList[obstacle_list_ind+1] - obstacleList[obstacle_list_ind];
    }

  } else {
    // Orange obstacle detection logic to be used when using orange detector
    //FOR EVERY ROW CHECK IF THE AMOUNT OF ORANGE IS ABOVE CERTAIN THRESHOLD
    for(uint16_t i=0; i<520;i++) {
      if (orangeincolumncounter[i] >= 60) {
        //YES! -> OBSTACLEINCOLUMN[i] = 1 
        obstacleincolumn[i] = 1;
      }
      else{
        //NO! -> NO OBSTACLE FOUND IN THIS ROW
        obstacleincolumn[i] = 0;
      }
    }
    //LOOPING THROUGH OBSTACLEINCOLUMN
    for (uint32_t ind=0; ind < 520; ind++) {
      //CHECKING WHERE WE GO FROM - TO 1 VALUE (BEGIN OF OBSTACLE)
      if ((obstacleincolumn[ind] == 0) && (obstacleincolumn[ind + 1] == 1)) {
        //STORE AS LEFT PIXEL
        left_pixel = ind + 1;
      }
      //CHECKING WHERE WE GO FROM 1 TO 0 VALUE (END OF OBSTACLE)
      if ((obstacleincolumn[ind] == 1) && (obstacleincolumn[ind + 1] == 0)) {
        //STORE AS RIGHT PIXEL
        right_pixel = ind;

        //CHECKING IF THE OBSTACLE IS WIDE ENOUGH TO BE INTERESTING
        if (right_pixel - left_pixel >= min_nrofCols) {
          //YES! -> ADDED TO OBSTACLELIST
          obstacleList[foundobstacles] = left_pixel;
          obstacleList[foundobstacles + 1] = right_pixel;
          obstacleList[foundobstacles + 2] = right_pixel - left_pixel + 1;
          foundobstacles += 3;
        }
      }
    }
  }
}
//DONE! NOW THE OBSTACLELIST IS FILLED WITH ALL THE FOUND OBSTACLES



//COMMUNICATE OBSTACLELIST TO ORANGE_AVOIDER.C (PROCESS OF THIS FILE ENDS HERE)
void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated)
  {

    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].left_pixel, local_filters[0].right_pixel,
        0, 0, 
        obstacleList[0],obstacleList[1],obstacleList[2],
        obstacleList[3],obstacleList[4],obstacleList[5],
        obstacleList[6],obstacleList[7],obstacleList[8],
        obstacleList[9],obstacleList[10],obstacleList[11],
        obstacleList[12],obstacleList[13],obstacleList[14], 0);
    local_filters[0].updated = false;
  }
}

bool isGreen_yuv(uint8_t *yp, uint8_t *up, uint8_t *vp, bool is_sim){
  // This function checks whether the given combination of YUV values makes an green pixel


  bool is_green = false;
  // Check whether we are running in a simulation, the values are slightly different for simulation and real world camerashots
  // All values are determined by training a decision tree classifier on a set of images where a mask indicating as a label outlining the objects with pixels that we want to classify as green
  
  if(is_sim){

    // Classification using simulation pixel values
    if ((*vp <= 134)&&(*up<=96)){
      is_green = true;
    }

  } else {
    
    // Classification using real world pixel values
    if((*up<=104)&&(*vp<=143)&&(*yp<=173)){
      is_green = true;
    } else if ((*up>104)&&(*up<=107)&&(*vp<=138)){
      is_green = true;
    }
  
  }

  return is_green;
}

bool isOrange_yuv(uint8_t *yp, uint8_t *up, uint8_t *vp, bool is_sim){
  // This function checks whether the given combination of YUV values makes an orange pixel

  bool is_orange = false;
  
  // Check whether we are running in a simulation, the values are slightly different for simulation and real world camerashots
  // All values are determined by training a decision tree classifier on a set of images where a mask indicating as a label outlining the objects with pixels that we want to classify as orange
  if (is_sim) {

    // Classification using simulation pixel values
    if (*vp >= 168){ 
      if (*vp <= 178){
        if (*yp > 70){
          is_orange = true;
        }
      }
      else if(*yp >= 80){
        is_orange = true;
      }
    }

  } else {

    // Classification using real world pixel values
    // if (*vp<150) {
    //   if ((*vp > 146) && (*up < 95)){
    //     is_orange = true;
    //   }
    // }
    // else {
    //   if (*up < 107) {
    //     is_orange = true;
    //   }
    //   else if (*vp > 172){
    //     is_orange = true;
    //   }
    // }
    if(*vp>148){
      if((*up<=99)&&(*yp>172)){
        is_orange = true;
      } else if((*up>99)&&(*vp>170)){
        is_orange = true;
      }
    }
  }
  
  return is_orange;
}
