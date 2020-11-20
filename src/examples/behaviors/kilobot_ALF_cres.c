#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "distribution_functions.c"

#define RESOURCES_SIZE 2

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
    UNCOMMITTED = 0,
    COMMITTED_N = 1,
    COMMITTED_S = 2,
} decision_t;

/*State*/
motion_t current_motion_type = STOP;            // Current motion type
decision_t current_decision_state = UNCOMMITTED;        // Current state

/*Random walk*/
const float std_motion_steps = 20*16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.0; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 120; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
uint32_t straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;

/*Communication*/
uint8_t sa_type = 0;                                //Variables for Smart Arena messages
uint32_t sa_payload_blue = 0;
uint32_t sa_payload_red = 0;
bool new_sa_msg = false;
uint8_t sent_message = 1;
uint8_t to_send_message = false;
message_t interactive_message;
message_t to_parse_message;

/*Decision making*/
const float tau = 1;
const float h = 0.1111111;
const float k = 0.8888889;
const uint16_t max_decision_ticks = 320; //10 secondi
uint32_t last_decision_ticks = 0;

uint8_t internal_error = 0;             //computation error
const float ema_alpha = 0.1;            //exponential moving average
uint8_t resources_pops[RESOURCES_SIZE]; //keep local knowledge about resources
uint8_t recruiter_state = UNCOMMITTED;  //commitment communicated by another robot

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d) {
  /* Unpack the message - extract ID, type and payload */
  /*type 0 is for messages coming from ARK*/
  if (msg->type == 0) {
      int id1 = msg->data[0];
      int id2 = msg->data[3];
      int id3 = msg->data[6];
      if (id1 == kilo_uid) {
          sa_payload_red = msg->data[1];
          sa_payload_blue = msg->data[2];
          new_sa_msg = true;
      }
      else if (id2 == kilo_uid) {
          sa_payload_red = msg->data[4];
          sa_payload_blue = msg->data[5];
          new_sa_msg = true;
      }
      else if (id3 == kilo_uid) {
          sa_payload_red = msg->data[7];
          sa_payload_blue = msg->data[8];
          new_sa_msg = true;
      }

      if (sa_type==1){
        resources_pops[0]= sa_payload_red;  //red resources
      }
      else if (sa_type==2)
      {
        resources_pops[1]= sa_payload_blue;  //blue resources
      }
  }
  /*type 1 is for messages coming from other kilobots*/
  else if(msg->type==1) {
    /* get id (always firt byte when coming from another kb) */
    uint8_t id = msg->data[0];
    // check that is a valid crc and another kb
    if(id!=kilo_uid){
      // store the message for later parsing to avoid the rx to interfer with the loop
      recruiter_state = msg->data[1];
    }
  }
  // /* For another kind of message */
  // else if (msg->type == 120) {
  //     int id = (msg->data[0] << 8) | msg->data[1];
  //     if (id == kilo_uid) {
  //         set_color(RGB(0,0,3));
  //     } else {
  //         set_color(RGB(0,3,0));
  //     }
  // }
}

/*-------------------------------------------------------------------*/
/* Message transmission                                              */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
  if(to_send_message) {
    /* this one is filled in the loop */
    to_send_message = false;

    return &interactive_message;
  } else {
    return NULL;
  }
}
void message_tx_success() {
  sent_message = 1;
}
/*-------------------------------------------------------------------*/
/* For Brodcast                                                      */
/*-------------------------------------------------------------------*/
/* Tell the kilobot to send its own state */
void send_own_state() {
    // fill my message before resetting the temp resource count
    // fill up message type. Type 1 used for kbs
    interactive_message.type = 1;
    // fill up the current kb id
    interactive_message.data[0] = kilo_uid;
    // fill up the current states
    interactive_message.data[1] = current_decision_state;
    // fill up the crc
    interactive_message.crc = message_crc(&interactive_message);
    // tell that we have a msg to send
    to_send_message = true;
    // avoid rebroadcast to overwrite prev message
    sent_message = 0;
}
/*-------------------------------------------------------------------*/
/* Decision Making Function                                          */
/* Sets ths current_decision var                                     */
/*-------------------------------------------------------------------*/
void take_decision() {
  if (kilo_ticks >= max_decision_ticks + last_decision_ticks){
    // temp variable used all along to account for quorum sensing
    uint8_t resource_index = 0;
    /* Start decision process */
    if(current_decision_state == UNCOMMITTED) {
      uint8_t commitment = 0;
      /****************************************************/
      /* spontaneous commitment process through discovery */
      /****************************************************/
      uint8_t random_resource = rand_soft()%RESOURCES_SIZE;
      // normalized between 0 and 255
      commitment = (uint8_t)floor(resources_pops[random_resource]*h*tau);
      /****************************************************/
      /* recruitment over a random agent            m_sData      */
      /****************************************************/
      uint8_t recruitment = 0;
      // if the recruiter is committed
      if(recruiter_state != UNCOMMITTED) {
        /* get the correct index in case of quorum sensing mechanism */
        resource_index = recruiter_state-1;
        // compute recruitment value for current agent
        recruitment = (uint8_t)floor(resources_pops[resource_index]*k*tau);
      }
      /****************************************************/
      /* extraction                                       */
      /****************************************************/
      /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
      if((uint16_t)commitment+(uint16_t)recruitment > 255) {
        internal_error = true;
        return;
      }
      // a random number to extract next decision
      uint8_t extraction = rand_soft();
      // if the extracted number is less than commitment, then commit
      if(extraction < commitment) {
        current_decision_state = random_resource+1;
        return;
      }
      // subtract commitments
      extraction = extraction - commitment;
      // if the extracted number is less than recruitment, then recruited
      if(extraction < recruitment) {
        current_decision_state = recruiter_state;
        return;
      }
      //update led
      if (current_decision_state = COMMITTED_N) {
        set_color(RGB(3,0,0));
      }
      if (current_decision_state = COMMITTED_S) {
        set_color(RGB(0,0,3));
      }
    }

    else {
      /****************************************************/
      /* cross inhibtion over a random agent              */
      /****************************************************/
      uint8_t cross_inhibition = 0;
      // if the inhibitor is committed or in quorum but not same as us
      if(recruiter_state != UNCOMMITTED && current_decision_state != recruiter_state){
        /* get the correct index in case of quorum sensing mechanism */
        resource_index = recruiter_state-1;
        // compute recruitment value for current agent
        cross_inhibition = (uint8_t)floor(resources_pops[resource_index]*k*tau);
      }
      /****************************************************/
      /* extraction                                       */
      /****************************************************/
      /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
      if((uint16_t)cross_inhibition > 255) {
        internal_error = true;
        return;
      }
      // a random number to extract next decision
      uint8_t extraction = rand_soft();
      // subtract cross-inhibition
      if(extraction < cross_inhibition) {
        current_decision_state = UNCOMMITTED;
        set_color(RGB(0,0,0));
        return;
      }
    }
    /* erase memory of neighbour commitment*/
    recruiter_state = UNCOMMITTED;
    last_decision_ticks = kilo_ticks;
    printf("deciding\n");
  }
}

/*-------------------------------------------------------------------*/
/* Set LED color for commitment */
/*-------------------------------------------------------------------*/
/*void update_led_status() {
    if(current_decision_state==0) 
      set_color(RGB(0,0,0));
    else if(current_decision_state==1) 
      set_color(RGB(3,0,0));
    else if(current_decision_state==2) 
      set_color(RGB(0,0,3));
}*/

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
        send_own_state();
        take_decision();
        //update_led_status();
}
int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    kilo_start(setup, loop);
    return 0;
}