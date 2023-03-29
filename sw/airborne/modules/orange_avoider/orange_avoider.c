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
static uint8_t increase_nav_heading_left_turn(float incrementDegrees);
static uint8_t increase_nav_heading_right_turn(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);
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
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count[15] ;                // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 10.f;          // heading angle increment [deg]
float maxDistance = 5;               // max waypoint displacement [m]


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
int safe_width = 2; //Safe_Distance_Width

void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  // chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}


// for(i=2; i <=14){
//   if (x[i] > collision_threshold || x[i] ==0)
//     {
//         //update confidence level 
//         // switch case to SAFE
//         printf("I am safe, i am moving forward!I am moving forward!");
//     }
//   else if(x[i] <= collision_threshold)
//   {
//     //switch case to OBSTACLE Found
//     if(x[i-2] < frame_center_coordinate) // this means that the obstabcle 
//                                           // is to the left of center line
//       {
//         //turn right
//         printf("Object at Left, Giving command to turn right");
//       }
//   }
//   i = i+3;
// }
int counter = 0;
void orange_avoider_periodic(void)
{
  uint8 bigoldobstacle[2] = {0,0}
  counter++;
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    printf("I am flying?");
    return;
  }
  // int color_count [15] = {1,3,4,6,8,1,12,16,2,0,0,0,0,0,0};
  
  
  printf("Counter : ------------%d-------------",counter);

  printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",color_count[0],color_count[1],color_count[2],
    color_count[3],color_count[4],color_count[5],color_count[6],color_count[7],color_count[8],
    color_count[9],color_count[10],color_count[11],color_count[12],color_count[13],color_count[14],
    color_count[15]); /// //Print all Incoming color coordinate array 

  for(int i=2; i <=14; i+= 3) //joep if obstacles are far enough don't do anything further logic is in the switch case
  {
      if (color_count[i] > collision_threshold){
      navigation_state = OBSTACLE_FOUND;
      }
  }

  // if(color_count_min)

    // bound obstacle_free_confidence JJJJJJJJJJJJ
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
    float moveDistance = fminf(maxDistance, 0.4f * obstacle_free_confidence);
    //float moveDistance = maxDistance;
    switch (navigation_state)
    {
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0 ){   /////////// Jagga: Need to change here
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }   
      /// J : Code came till here
      break;
    case OBSTACLE_FOUND:
      uint16_t biggestgap;
      int16_t angleincrement;
      //TInka logic finding biggest gap
      angleincrement = (biggestgap - 260)

      }


      break;

    case SEARCH_FOR_SAFE_HEADING:
    
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
    
     case OUT_OF_BOUNDS:
       
    //   VERBOSE_PRINT("I am in the OUt of bounds Case");
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);
        // reset safe counter
        // obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;

    default:

      break;
  }
  
  return;
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


// uint8_t increase_nav_heading_right_turn(float incrementDegrees)
// {
//   float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(-1*incrementDegrees);

//   // normalize heading to [-pi, pi]
//   FLOAT_ANGLE_NORMALIZE(new_heading);

//   // set heading, declared in firmwares/rotorcraft/navigation.h
//   // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
//   nav_heading = ANGLE_BFP_OF_REAL(new_heading);

//   VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
//   return false;
// }

// uint8_t increase_nav_heading_left_turn(float incrementDegrees)
// {
//   float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

//   // normalize heading to [-pi, pi]
//   FLOAT_ANGLE_NORMALIZE(new_heading);

//   // set heading, declared in firmwares/rotorcraft/navigation.h
//   // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
//   nav_heading = ANGLE_BFP_OF_REAL(new_heading);

//   VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
//   return false;
// }
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
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
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