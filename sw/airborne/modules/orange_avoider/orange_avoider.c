/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance_Right(void);
uint8_t chooseRandomIncrementAvoidance_Left(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE; // start assuming it is safe
int32_t color_count[15] ;                // orange color count from color filter for obstacle detection
float heading_increment = 10.f;          // heading angle increment [deg]
float maxDistance = 5;               // max waypoint displacement [m]
float unitdistance = 150.111069;


const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality1,int32_t quality2,int32_t quality3,int32_t quality4,int32_t quality5,int32_t quality6,int32_t quality7,int32_t quality8,int32_t quality9,int32_t quality10,int32_t quality11,int32_t quality12,int32_t quality13,int32_t quality14,int32_t quality15, int16_t __attribute__((unused)) extra)
{
color_count[0] = quality1;
color_count[1] = quality2;
color_count[2] = quality3;
color_count[3] = quality4;
color_count[4] = quality5;
color_count[5] = quality6;
color_count[6] = quality7;
color_count[7] = quality8;
color_count[8] = quality9;
color_count[9] = quality10;
color_count[10] = quality11;
color_count[11] = quality12;
color_count[12] = quality13;
color_count[13] = quality14;
color_count[14] = quality15;
}

int collision_threshold = 20; // Minial collison avoidance distance (in m) 
int frame_center_coordinate = 265; // Safe_center_Coordinate

void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  // chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

bool wecangoleft = true;

void orange_avoider_periodic(void)
{
    int16_t biggestgap_signed;
    int16_t dx = 0;
    int biggestgap= 0;
    int valuebiggestgap = 0;
    float headingchange;
    int right;
    int left;
    int nrofobstacles = 0;

  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    printf("\n IM NOT FLYING");
    return;
  }

    for(int i=2; i <=15; i+= 3) //joep if obstacles are far enough don't do anything further logic is in the switch case
    {
        if (color_count[i] > collision_threshold*2){
            //printf("\n I FOUND A BIG OBSTACLE");
            //TInka logic finding biggest gap
            //find the amount of obstacles detected
            for (int loop = 0; loop < 5; loop++){
                if (color_count[(loop)*3+2] != 0){
                    nrofobstacles ++;
                }
            }
            //printf("\n %d OF 'M!!", nrofobstacles);

            for (int loop = 0; loop<(nrofobstacles + 1); loop++){
                if (nrofobstacles == 0){
                    valuebiggestgap = 520;
                    biggestgap = 260;

                }else{

                    //loop for first gap, because the object might already start at 0
                    if (loop == 0){
                        biggestgap = color_count[0]/2;
                        valuebiggestgap = color_count[0];
                    }else{
                        left = color_count[3*(loop-1)+1];
                        right = color_count[3*(loop-1)+3];
                        //loop for last gap, because the object might end at 520
                        if (loop == nrofobstacles) {
                            //check if gap is bigger than the previously detected one
                            if ((520 - left) >= valuebiggestgap){
                                //import values to the biggest gap ones
                                biggestgap = left + (520-left)/2;
                                valuebiggestgap = 520 - left;
                            }

                            //calculation of all the middle gaps
                        }else{
                            //again check if the gap is bigger than the previous detected one
                            if ((right-left) >= valuebiggestgap){
                                //import values to the biggest gap ones
                                biggestgap = left + (right-left)/2;
                                valuebiggestgap = right - left;
                            }

                        }}}
            }
            biggestgap_signed = (int16_t) biggestgap;
            dx = (biggestgap_signed - 260);
            if(abs(dx)>20){
                //printf("\n GAP IS FAR TO MY RIGHT OR LEFT: %d", dx);
                navigation_state = OBSTACLE_FOUND;

            }else{
                wecangoleft = true;
                //printf("\n GAP IS NOT SIGNIFICANT going to save mode ");
                navigation_state = SAFE;

            }
            break;
        }
    }

    // bound obstacle_free_confidence JJJJJJJJJJJJ
    float trajectorydistance = 2.5;
    switch (navigation_state)
    {

    case SAFE:
      //printf("\n IM SAFE");
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, trajectorydistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
        moveWaypointForward(WP_GOAL, trajectorydistance);
      }   
      /// J : Code came till here
      break;

    case OBSTACLE_FOUND:
      //printf("\n IM NOW GOING TO THE GAP");
      //Joep cast biggest gap to a signed int
      headingchange = atanf((float) dx / unitdistance) * 20;
      //printf("\n PRINTING HEADING CHANGE : %f", headingchange);
      if (headingchange > 0){
          //printf("\n WE CAN GO LEFT AGAIN");
          wecangoleft = true;
      }
      if (wecangoleft) {
          //printf("\n something wrong");
          increase_nav_heading(headingchange);
          moveWaypointForward(WP_TRAJECTORY, 2.5f);
          moveWaypointForward(WP_TRAJECTORY, 2.5f);
      }
      else {
          //printf("\n WE CANNOT GO LEFT :(");
          increase_nav_heading(30);
      }
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          navigation_state = OUT_OF_BOUNDS;
      }
      else {
          navigation_state = SAFE;
      }
            break;

    case SEARCH_FOR_SAFE_HEADING:
        navigation_state = SAFE; //wont reach this at this point
        break;
    
    case OUT_OF_BOUNDS:
        //printf("\n I AM OUT OF BOUNDS :(");
        //   VERBOSE_PRINT("I am in the OUt of bounds Case");
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          increase_nav_heading(20);
          moveWaypointForward(WP_TRAJECTORY, 2.5f);
          wecangoleft = false;
          //printf("I CAN NO LONGER GO LEFT :(");
          }
      else{
          //printf("\n YAY IM NO LONGER OUT OF BOUNDS");
          moveWaypointForward(WP_TRAJECTORY, 2.5f);
          navigation_state = SAFE;
      }



      break;

    default:
      break;
  }
}



/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}


/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}


uint8_t chooseRandomIncrementAvoidance_Right(void)
{
  // Randomly choose CW avoiding direction
    heading_increment = 10.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n Right", heading_increment);
  return false;
}

uint8_t chooseRandomIncrementAvoidance_Left(void)
{
  // Randomly choose CCW avoiding direction
    heading_increment = -10.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n Left", heading_increment);
  return false;
}