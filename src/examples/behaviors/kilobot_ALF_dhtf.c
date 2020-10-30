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

int sa_type = 3;                                //Variables for Smart Arena messages
int sa_payload = 0;

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
        }
        if (id2 == kilo_uid) {
            sa_type = msg->data[4] >> 2 & 0x0F;
            sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
        }
        if (id3 == kilo_uid) {
            sa_type = msg->data[7] >> 2 & 0x0F;
            sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
        }

        location = sa_type;
        timeout_param = sa_payload;
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
}


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
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

/*-------------------------------------------------------------------*/
/* Function implementing kilobot FSM and state transitions           */
/*-------------------------------------------------------------------*/
void finite_state_machine(){
    /* State transition */
    switch (current_state) {
        case RANDOM_WALKING : {
            //set_color(RGB(0,0,0));
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
                set_color(RGB(0,0,0));
            }
            /* Timeout condition */
            timeout--;
            if (timeout == 0) {
                set_color(RGB(3,0,0));
                current_state = LEAVING;
                //leaving_timer =150;
                set_motion(FORWARD);
            }
            break;
        }
        case LEAVING : {        /*se vogliamo un solo messaggio: basta togliere il timer, appena arriva un msg OUTSIDE il robot riparte ad esplorare (spegne il led)*/
            // if (leaving_timer>0){
            //     //set_motion(FORWARD);
            //     leaving_timer--;
            // }
            // else{
                if(location == OUTSIDE){
                    current_state = RANDOM_WALKING;
                    set_color(RGB(0,0,0));
                    //(FORWARD);
                }
            //     else{
            //         turn_timer = 5;
            //         current_state = ROTATION;
            //         set_motion (TURN_LEFT);
            //     }
            //}
            break;
        }
        // case ROTATION : {
        //     //set_color(RGB(3,3,3));
        //     set_motion (TURN_LEFT);
        //     if(turn_timer==0){
        //         current_state = LEAVING;
        //         set_motion(FORWARD);
        //         leaving_timer=50;
        //     }
        //     turn_timer--;
        //     break;
        // }
    }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
        random_walk();
        finite_state_machine();
        //printf("stato %d\n",current_state);
}

int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);
    return 0;
}