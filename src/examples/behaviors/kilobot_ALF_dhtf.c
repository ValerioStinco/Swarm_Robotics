#include "kilolib.h"
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include "distribution_functions.c"

typedef enum {  // Enum for different motion types
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    STOP = 3,
    FORWARD = 4,
} motion_t;


typedef enum {  // Enum for boolean flags
    false = 0,
    true = 1,
} bool;

typedef enum {  // Enum for the robot states
    RANDOM_WALKING = 0,
    WAITING = 1,
    LEAVING = 2,
    ROTATION = 3,
} action_t;

typedef enum {  // Enum for the robot position wrt to areas
    OUTSIDE = 0,
    INSIDE = 1,
} position_t;

motion_t current_motion_type = STOP;            // Current motion type

action_t current_state = RANDOM_WALKING;        // Current state

/* SALAH---------------------------------------------- */
// uint32_t last_turn_ticks = 0;                   // Counters for motion, turning and random_walk
// uint32_t turn_ticks = 60;
// unsigned int turning_ticks = 0;
// const uint8_t max_turning_ticks = 160;          // Constant to allow a maximum rotation of 180 degrees with \omega=\pi/5
// const uint16_t max_straight_ticks = 320;        // Set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32)
// uint32_t last_motion_ticks = 0;
// uint32_t turn_into_random_walker_ticks = 160;   // Timestep to wait without any direction message before turning into random_walker
// uint32_t last_direction_msg = 0;
/* LUIGI---------------------------------------------- */
int location=0;
int timeout_param=0;
const float std_motion_steps = 20*16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.0; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;
/* ---------------------------------------------- */

int sa_type = 3;                                //Variables for Smart Arena messages
int sa_payload = 0;
bool new_sa_msg = false;

int timeout;                                    //Internal counter for task complention wait
int leaving_timer;                              //Internal counter for the action of leaving a task when timeout expires
int turn_timer;                                //Avoid the robot to get stuck in Leagving

/* PARAMETER: change this value to determine timeout length */
const int TIMEOUT_CONST = 500;

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    bool calibrated = true;
    if ( current_motion_type != new_motion_type ){
        switch( new_motion_type ) {
        case FORWARD:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_straight_left,kilo_straight_right);
            else
                set_motors(70,70);
            break;
        case TURN_LEFT:
            spinup_motors();
            if (calibrated)
                set_motors(kilo_turn_left,0);
            else
                set_motors(70,0);
            break;
        case TURN_RIGHT:
            spinup_motors();
            if (calibrated)
                set_motors(0,kilo_turn_right);
            else
                set_motors(0,70);
            break;
        case STOP:
        default:
            set_motors(0,0);
        }
        current_motion_type = new_motion_type;
    }
}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d) {

    /* Unpack the message - extract ID, type and payload */
    if (msg->type == 0) {
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
        if (id1 == kilo_uid) {
            sa_type = msg->data[1] >> 2 & 0x0F;
            sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
            new_sa_msg = true;
        }
        if (id2 == kilo_uid) {
            sa_type = msg->data[4] >> 2 & 0x0F;
            sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
            new_sa_msg = true;
        }
        if (id3 == kilo_uid) {
            sa_type = msg->data[7] >> 2 & 0x0F;
            sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
            new_sa_msg = true;
        }
    }

    /* For another kind of message */
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(0,3,0));
        }
    }

    location=sa_type;
    timeout_param=sa_payload;
}


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/

//***Random walk di Salah***
// void random_walk() {
//     switch( current_motion_type ) {
//     case TURN_LEFT:
//         if( kilo_ticks > last_motion_ticks + turning_ticks ) {
//             /* start moving forward */
//             last_motion_ticks = kilo_ticks;  // fixed time FORWARD
//             //last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
//             set_motion(FORWARD);
//         }
//     case TURN_RIGHT:
//         if( kilo_ticks > last_motion_ticks + turning_ticks ) {
//             /* start moving forward */
//             last_motion_ticks = kilo_ticks;  // fixed time FORWARD
//             //last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
//             set_motion(FORWARD);
//         }
//         break;
//     case FORWARD:
//         if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
//             /* perform a random turn */
//             last_motion_ticks = kilo_ticks;
//             if( rand()%2 ) {
//                 set_motion(TURN_LEFT);
//             }
//             else {
//                 set_motion(TURN_RIGHT);
//             }
//             turning_ticks = rand()%max_turning_ticks + 1;
//         }
//         break;
//     case STOP:
//     default:
//         set_motion(FORWARD);
//     }
// }

//***Random walk di Luigi***
void random_walk()
{
  switch (current_motion_type) 
  {
  case TURN_LEFT:
  case TURN_RIGHT:
    /* if turned for enough time move forward */
    if (kilo_ticks > last_motion_ticks + turning_ticks) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;

    case FORWARD:
    /* if moved forward for enough time turn */
    if (kilo_ticks > last_motion_ticks + straight_ticks) {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      
      if (rand_soft() % 2)
      {
        set_motion(TURN_LEFT);
      }
      else
      {
        set_motion(TURN_RIGHT);
      }
      double angle = 0;
      if(crw_exponent == 0) 
      {
        angle = (uniform_distribution(0, (M_PI)));
      }
      else
      {
        angle = fabs(wrapped_cauchy_ppf(crw_exponent));
      }
      turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
    break;

  case STOP:
  default:
    set_motion(STOP);
  }
}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup() {
    /* Initialise LED and motors */
    set_color(RGB(0,0,0));
    set_motors(0,0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    /* Initialise motion variables */
    last_motion_ticks = rand()%max_straight_ticks;
    set_motion(FORWARD);
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
void check_state(){
    /* State transition */
    switch (current_state) {
        case RANDOM_WALKING : {
            set_color(RGB(0,0,0));
            if(location == INSIDE){
                timeout = timeout_param*TIMEOUT_CONST;
                current_state = WAITING;
                set_motion(STOP);
            }
            break;
        }
        case WAITING : {
            set_color(RGB(0,3,0));
            if(location == OUTSIDE){
                current_state = RANDOM_WALKING;
                set_motion(FORWARD);
            }
            /* Timeout condition */
            timeout--;
            if (timeout == 0) {
                set_color(RGB(3,0,0));
                current_state = LEAVING;
                leaving_timer =50;
                set_motion(FORWARD);
            }
            break;
        }
        case LEAVING : {
            set_color(RGB(3,0,0));
            if (leaving_timer>0){
                set_motion(FORWARD);
                leaving_timer--;
            }
            else{
                if(location == OUTSIDE){
                    current_state = RANDOM_WALKING;
                    set_color(RGB(0,0,0));
                    set_motion(FORWARD);
                }
                else{
                    turn_timer = 5;
                    current_state = ROTATION;
                    set_motion (TURN_LEFT);
                    /*set_motion(TURN_LEFT);
                    leaving_timer=50;
                    break_timer--;
                    if (break_timer == 0){
                        current_state = RANDOM_WALKING;
                        set_motion(FORWARD);  
                    }*/
                }
            }
            break;
        }
        case ROTATION : {
            //set_color(RGB(3,3,3));
            set_motion (TURN_LEFT);
            if(turn_timer==0){
                current_state = LEAVING;
                set_motion(FORWARD);
                leaving_timer=50;
            }
            turn_timer--;
            break;
        }
    }
}
    ///////////////////////////////////////////////
    ////////////////////////////////////////////////
/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
        random_walk();
        check_state();
        //printf("stato %d\n",current_state);
}

int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);
    return 0;
}