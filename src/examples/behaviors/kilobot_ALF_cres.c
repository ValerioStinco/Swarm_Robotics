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
    UNCOMMITTED = 0,
    COMMITTED_N = 1,
    COMMITTED_S = 2,
} action_t;

typedef enum {  // Enum for the robot position wrt to areas
    OUTSIDE = 0,
    INSIDE = 1,
} position_t;

motion_t current_motion_type = STOP;            // Current motion type

action_t current_state = UNCOMMITTED;        // Current state

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
const uint8_t max_turning_ticks = 120; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;
/* ---------------------------------------------- */

int sa_type = 0;                                //Variables for Smart Arena messages
int sa_payload = 0;
bool new_sa_msg = false;

/*informations received from ARK*/
int imposed_direction = 0;                      //which direction to point at
int current_kb_angle=0;                         //current orientation of the robot

int straight_timer;                             //time of straight walk toward target
int turn_timer;                                 //time of turning toward target
double directed_motion_freq=0.01;               //frequency of motion toward target

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

    imposed_direction=sa_type;
    current_kb_angle=sa_payload;
//////////////////////////////////////////////////////
da aggiungere messaggi con altri kb
/////////////////////////////////////////////////////
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
/* Decision Making Function                                          */
/* Sets ths current_decision var                                     */
/*-------------------------------------------------------------------*/
void take_decision() {
  // temp variable used all along to account for quorum sensing
  uint8_t resource_index = 0;
  /* Start decision process */
  if(current_decision_state == NOT_COMMITTED) {
    uint8_t commitment = 0;
    /****************************************************/
    /* spontaneous commitment process through discovery */
    /****************************************************/
    // if over umin threshold
    uint8_t random_resource = rand_soft()%RESOURCES_SIZE;
    // normalized between 0 and 255
    commitment = (uint8_t)floor(resources_pops[random_resource]*h*tau);
    /****************************************************/
    /* recruitment over a random agent                  */
    /****************************************************/
    uint8_t recruitment = 0;
    node_t* recruitment_message = NULL;
    uint8_t recruiter_state = 255;
    // if list non empty (computed in the clean right after the call to this)
    if(list_size > 0) {
      uint8_t rand = rand_soft()%list_size;
      recruitment_message = b_head;
      while(recruitment_message && rand) {
        // set recruiter state
        recruitment_message = recruitment_message->next;
        rand = rand-1;
      }
      recruiter_state = recruitment_message->msg.data[1];
    }
    // if the recruiter is committed
    if(recruiter_state != NOT_COMMITTED) {
      /* get the correct index in case of quorum sensing mechanism */
      resource_index = recruiter_state;
      // compute recruitment value for current agent
      recruitment = (uint8_t)floor(resources_pops[resource_index]*k*tau);
    }
    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                              */
    if((uint16_t)commitment+(uint16_t)recruitment > 255) {
      internal_error = true;
      return;
    }
    // a random number to extract next decision
    uint8_t extraction = rand_soft();
    // if the extracted number is less than commitment, then commit
    if(extraction < commitment) {
      current_decision_state = random_resource;
      return;
    }
    // subtract commitments
    extraction = extraction - commitment;
    // if the extracted number is less than recruitment, then recruited
    if(extraction < recruitment) {
      current_decision_state = recruiter_state;
      return;
    }
  }

  else {
    /****************************************************/
    /* cross inhibtion over a random agent              */
    /****************************************************/
    uint8_t cross_inhibition = 0;
    node_t *cross_message = NULL;
    uint8_t inhibitor_state = 255;
    // get list size
    uint16_t list_size = mtl_size(b_head);
    // if list non empty
    // if list non empty (computed in the clean right after the call to this)
    if(list_size > 0) {
      uint8_t rand = rand_soft()%list_size;
      cross_message = b_head;
      while(cross_message && rand) {
        // set recruiter state
        cross_message = cross_message->next;
        rand = rand-1;
      }
      inhibitor_state = cross_message->msg.data[1];
    }
    // if the inhibitor is committed or in quorum but not same as us
    if(inhibitor_state != NOT_COMMITTED &&
       current_decision_state != inhibitor_state){
      /* get the correct index in case of quorum sensing mechanism */
      resource_index = inhibitor_state;
      // compute recruitment value for current agent
      cross_inhibition = (uint8_t)floor(resources_pops[resource_index]*k*tau);
    }
    /****************************************************/
    /* extraction                                       */
    /****************************************************/
    /* check if the sum of all processes is below 1 (here 255 since normalize to uint_8) */
    /*                                  STOP                                             */
    if((uint16_t)cross_inhibition > 255) {
      internal_error = true;
      return;
    }
    // a random number to extract next decision
    uint8_t extraction = rand_soft();
    // subtract abandon
    // subtract cross-inhibition
    if(extraction < cross_inhibition) {
      current_decision_state = NOT_COMMITTED;
      return;
    }
  }
}

/* Set LED color for commitment */
void update_led_status() {
    if(current_decision_state==0) 
      set_color(RGB(0,0,0));
    else if(current_decision_state==1) 
      set_color(RGB(3,0,0));
    else if(current_decision_state==2) 
      set_color(RGB(0,3,0));
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
        update_led_status();
}
int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_message_tx = message_tx;
    kilo_start(setup, loop);
    return 0;
}