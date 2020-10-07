#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
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
    TURN_TO_TARGET = 1,
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
const float std_motion_steps = 20*16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.0; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;
/* ---------------------------------------------- */

int sa_type = 0;                                //Variables for Smart Arena messages
int sa_payload = 0;
bool new_sa_msg = false;

int best_side=0;                                //Direction where to direct the robot
int act_side;                                   //actual half of arena where the robot currently is
int straight_timer;                             //time of straight walk toward target
int turn_timer;                                 //time of turning toward target
double directed_motion_freq=0.01;                  //frequency of motion toward target

/* PARAMETER: change this value to determine timeout lenght */
int TIMEOUT_CONST = 500;

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

    /* Best side update */
    switch(sa_type){
        case 0:{
            act_side=1;
            //best_side=0;       //do not update if it wuold be set to 0
            break;
        }
        case 1:{
            act_side=1;
            best_side=1;
            break;
        }
        case 2:{
            act_side=1;
            best_side=2;
            break;
        }
        case 4:{
            act_side=2;
            //best_side=0;       //do not update if it wuold be set to 0
            break;
        }
        case 5:{
            act_side=2;
            best_side=1;
            break;
        }
        case 6:{
            act_side=2;
            best_side=2;
            break;
        }
    }
    // if(sa_type!=0){
    //     best_side=sa_type;  // 0:no preference;  1:up;  2::down;
    //     printf("TYPE: %d\n", sa_type);
    // }

    /* State transition */
    switch (current_state) {
        case RANDOM_WALKING : {
            set_color(RGB(0,0,0));
            printf("TYPE: %d\n", sa_type);
            //printf("SA_PAYLOAD: %d\n",sa_payload);
            if(act_side!=best_side){
                if((((double) rand() / (RAND_MAX))<directed_motion_freq) && (best_side!=0)){
                    current_state = TURN_TO_TARGET;
                    printf("TURN PERCHE BEST_SIDE è %d\n",best_side);
                    printf("INVECE ACT_SIDE è %d\n",act_side);
                }
            }
            break;
        }
        case TURN_TO_TARGET : {
            set_color(RGB(3,3,3));
            //printf("BIANCO");
            //printf("SA_PAYLOAD: %d\n",sa_payload);
            if((best_side==1)){           //VERSO L'ALTO
                if(((sa_payload>=0)&&(sa_payload<=10))||((sa_payload>=100)&&(sa_payload<=110))){
                    current_state = RANDOM_WALKING;
                    printf("AVVISO USCITA");
                }
                else{
                    if(sa_payload<100){   
                        set_motion (TURN_RIGHT);
                        //printf("RIGHT\n");
                    }
                    else{
                        set_motion (TURN_LEFT);
                        //printf("LEFT\n");
                    }
                }
            }
            else if((best_side==2)){      //VERSO IL BASSO
                if(((sa_payload>=20)&&(sa_payload<=31))||((sa_payload>=120)&&(sa_payload<=131))){
                    current_state = RANDOM_WALKING;
                    printf("AVVISO USCITA");
                }
                else{
                    if(sa_payload>100){   
                        set_motion (TURN_RIGHT);
                        printf("RIGHT\n");
                    }
                    else{
                        set_motion (TURN_LEFT);
                        printf("LEFT\n");
                    }
                }
            }
            //set_motion (TURN_RIGHT);
            break;
        }
    }
}


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
  switch (current_motion_type) {
  case TURN_LEFT:
  case TURN_RIGHT:
    /* if turned for enough time move forward */
    if(kilo_ticks > last_motion_ticks + turning_ticks) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;

  case FORWARD:
    /* if moved forward for enough time turn */
    if(kilo_ticks > last_motion_ticks + straight_ticks) {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      if (rand_soft() % 2) {
        set_motion(TURN_LEFT);
      } else {
        set_motion(TURN_RIGHT);
      }
      double angle = 0;
      /* random angle */
      if(crw_exponent == 0) {
        angle = uniform_distribution(0, (M_PI));
      } else {
        angle = fabs(wrapped_cauchy_ppf(crw_exponent));
      }

      /* compute turning time */
      turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
    break;

  case STOP:
  default:
    set_motion(FORWARD);
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
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
        random_walk();
}

int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);
    return 0;
}