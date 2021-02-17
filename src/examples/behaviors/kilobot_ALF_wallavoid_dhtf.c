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
    PARTY = 3,
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
/***********WALK PARAMETERS***********/
const float std_motion_steps = 20*16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 1.4; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.9; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;


// the kb is biased toward the center when close to the border
float rotation_to_center = 0; // if not 0 rotate toward the center (use to avoid being stuck)
uint8_t rotating = 0; // variable used to cope with wall avoidance (arena borders)

/* ---------------------------------------------- */

int sa_type = 3;                                //Variables for Smart Arena messages
int sa_payload = 0;

uint8_t start = 0;   // waiting from ARK a start signal to run the experiment 0 : not received, 1 : received, 2 : not need anymore to receive
int location=0;
int internal_timeout=0;                         //Internal counter for task complention wait
int turn_timer;                                //Avoid the robot to get stuck in Leagving

/* PARAMETER: change this value to determine timeout length */
const int TIMEOUT_CONST = 1;
const uint32_t to_sec = 32;
uint32_t last_waiting_ticks = 0;

uint32_t party_ticks = 0;

bool collision_avoidance_test = false;

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type) {
  if(current_motion_type != new_motion_type ) 
  {
    switch( new_motion_type ) {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left,kilo_straight_right);
      rotating = 0;
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left,0);
      rotating = 1;
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0,kilo_turn_right);
      rotating = 1;
      break;
    case STOP:
    default:
      set_motors(0,0);
      rotating = 0;
    }
    current_motion_type = new_motion_type;
  }
}

/*-------------------------------------------------------------------*/
/* Parse ARK received messages                                       */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index) 
{
  // index of first element in the 3 sub-blocks of data
  uint8_t shift = kb_index*3;

  sa_type = data[shift+1] >> 2 & 0x0F;
  sa_payload = ((data[shift+1]&0b11)  << 8) | (data[shift+2]);
  
  switch( sa_type ) {
    case 0:
      location = sa_type;
      if(sa_payload !=0 && rotation_to_center == 0)
      {
        // get rotation toward the center (if far from center)
        // avoid colliding with the wall
        uint8_t rotation_slice = sa_payload;
        if(rotation_slice == 3) 
        {
          rotation_to_center = -M_PI/3;
        } 
        else 
        {
          rotation_to_center = (float)rotation_slice*M_PI/3; 
        }
      }
      break;

    case 1:
      location = sa_type;
      internal_timeout = sa_payload * TIMEOUT_CONST * 10;
      break;
    
    case 2:
      if(sa_payload !=0 && rotation_to_center == 0)
      {
        // get rotation toward the center (if far from center)
        // avoid colliding with the wall
        uint8_t rotation_slice = sa_payload;
        if(rotation_slice == 3) 
        {
          rotation_to_center = -M_PI/3;
        } 
        else 
        {
          rotation_to_center = (float)rotation_slice*M_PI/3; 
        }
      }
      break;
    
    case 3:
      current_state = PARTY;
      party_ticks = kilo_ticks;
      break;
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
        
        if(id1 == kilo_uid) {
          parse_smart_arena_message(msg->data, 0);
        } 
        else if(id2 == kilo_uid) {
          parse_smart_arena_message(msg->data, 1);
        } 
        else if (id3 == kilo_uid) {
          parse_smart_arena_message(msg->data, 2);
        }

    }

    /* start signal!*/
    else if (msg->type == 1 && start != 2)
    {
      start = 1;
    }
    /* For assign id message */
    else if (msg->type == 120) 
    {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
}


/*-------------------------------------------------------------------*/
/* Function implementing the LMCRW random walk                       */
/*-------------------------------------------------------------------*/

void random_walk()
{
  /* if the arena signals a rotation, then rotate toward the center immediately */
  if(rotation_to_center != 0 && !rotating && internal_timeout == 0) {
    if(rotation_to_center > 0) {
      set_motion(TURN_LEFT);
    } else {
      set_motion(TURN_RIGHT);
    }
    // when too close to the border bias toward center
    float angle = abs(rotation_to_center);
    rotation_to_center = 0;
    /* compute turning time */
    turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
    straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    
    //set to true to test collision avoidance procedure
    // collision_avoidance_test = true;
    // set_color(RGB(0,3,0));
    
    return;
  }



  /* else keep on with normal random walk */
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

    set_motion(STOP);
}

/*-------------------------------------------------------------------*/
/* Function implementing kilobot FSM and state transitions           */
/*-------------------------------------------------------------------*/
void finite_state_machine(){
    /* State transition */
    switch (current_state) {
        case RANDOM_WALKING : {
            if(location == INSIDE){
                set_motion(STOP);

                last_waiting_ticks = kilo_ticks;
                
                set_color(RGB(3,0,0));
                current_state = WAITING;
            }
            break;
        }
        case WAITING : {
            if(location == OUTSIDE){
                set_motion(FORWARD);
                
                internal_timeout = 0;
                current_state = RANDOM_WALKING;
                set_color(RGB(0,0,0));
            }
            /* Timeout condition */
            if(kilo_ticks > last_waiting_ticks + internal_timeout * to_sec)
            {
		            internal_timeout = 0;

                set_motion(FORWARD);

                current_state = LEAVING;
                set_color(RGB(0,0,3));
            }
            break;
        }
        case LEAVING : {
            if(location == OUTSIDE){
                current_state = RANDOM_WALKING;
                set_color(RGB(0,0,0));
            }
            break;
        }
        case PARTY : {
          set_motion(STOP);
          set_color(RGB(0,3,0));
          delay(500);
          set_color(RGB(3,0,3));
          delay(500);
          if(kilo_ticks > party_ticks + 10 * to_sec)
          {
            set_motion(FORWARD);
            set_color(RGB(0,0,0));
            current_state = RANDOM_WALKING;
          }
          break;  
        }
    }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
        if(start == 1)
        {
          /* Initialise motion variables */
          last_motion_ticks = rand()%max_straight_ticks;
          set_motion(FORWARD);
          start = 2;
        }

        random_walk();
        finite_state_machine(); 

        if( collision_avoidance_test == true && kilo_ticks > last_motion_ticks + turning_ticks )
        {
          collision_avoidance_test = false;
          set_color(RGB(0,0,0));
        }
}

int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);
    return 0;
}
