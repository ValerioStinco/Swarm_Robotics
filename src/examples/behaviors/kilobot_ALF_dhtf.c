#include "kilolib.h"
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include "distribution_functions.c"

#define COLLISION_BITS 8
#define SECTORS_IN_COLLISION 2

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

typedef enum {  // Enum for the robot position wrt to areas
    LEFT = 1,
    RIGHT = 2,
} Free_space;

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
const float levy_exponent = 2; //LUIGI: 1.4; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.3; //LUIGI: 0.9; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 120; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;


/***********WALL AVOIDANCE***********/
// the kb is "equipped" with a proximity sensor

const uint8_t sector_base = (pow(2, COLLISION_BITS/2) - 1);  // an int of all ones to retrieve the left and then the right proximity sensor
uint8_t left_side = 0;
uint8_t right_side = 0;
uint8_t proximity_sensor = 0;
Free_space free_space = LEFT;
bool wall_avoidance_start = false;

/* ---------------------------------------------- */
//Variables for Smart Arena messages
int sa_type = 0;  
int sa_payload = 0;

int location=0;
int internal_timeout=0;                         //Internal counter for task complention wait
int turn_timer;                                //Avoid the robot to get stuck in Leagving

/* PARAMETER: change this value to determine timeout length */
const int TIMEOUT_CONST = 1;
const uint32_t to_sec = 32;
uint32_t last_waiting_ticks = 0;

uint32_t party_ticks = 0;


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
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left,0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0,kilo_turn_right);
      break;
    case STOP:
    default:
      set_motors(0,0);
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
    //   printf("outside\n");
      if(sa_payload !=0)
      {
        // get rotation toward the center (if far from center)
        // avoid colliding with the wall
        proximity_sensor = sa_payload;
        // printf("out sensor %d\n",proximity_sensor);
        wall_avoidance_start = true;
      }
      break;

    case 1:
      location = sa_type;
      internal_timeout = sa_payload * TIMEOUT_CONST * 10;
      break;
    
    case 2:
      if(sa_payload !=0)
      {
        // get rotation toward the center (if far from center)
        // avoid colliding with the wall
        proximity_sensor = sa_payload;
        // printf("leaving sensor %d\n",proximity_sensor);
        wall_avoidance_start = true;
        
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
        //   printf("Received somethingh\n");
          parse_smart_arena_message(msg->data, 0);
        } 
        else if(id2 == kilo_uid) {
        //   printf("Received somethingh\n");
          parse_smart_arena_message(msg->data, 1);
        } 
        else if (id3 == kilo_uid) {
        //   printf("Received somethingh\n");
          parse_smart_arena_message(msg->data, 2);
        }

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

    set_motion(FORWARD);
}

/*-------------------------------------------------------------------*/
/* count 1s after decimal to binary conversion                       */
/*-------------------------------------------------------------------*/
uint8_t countOnes(uint8_t n) 
{ 
  uint8_t count = 0;
  // array to store binary number 
  // uint8_t binaryNum[8]; 

  // counter for binary array 
  int i = 0; 
  while (n > 0) { 

      // storing remainder in binary array 
      // binaryNum[i] = n % 2; 
      if((n % 2) == 1)
        count++;
      n = n / 2; 
      i++; 
  } 
  
  return count;
} 

/*-------------------------------------------------------------------*/
/* Function implementing wall avoidance procedure                    */
/*-------------------------------------------------------------------*/
void wall_avoidance_procedure(uint8_t sensor_readings)
{
  right_side = sensor_readings & sector_base;
  left_side = (sensor_readings >> (COLLISION_BITS/2)) & sector_base;

  // printf("kID %d : ", kilo_uid);
  // printBits(sensor_readings, 8);
  
  // printf("\n");
  // printf("left: ");
  // printBits(left_side, 4);
  // printf("\n");
  // printf("right: ");
  // printBits(right_side, 4);
  
  // printf("\n");
  // printf("\n");


  uint8_t count_ones = countOnes(sensor_readings);
  if(count_ones > SECTORS_IN_COLLISION)
  {
    last_motion_ticks = kilo_ticks;
    turning_ticks = (uint32_t)((M_PI/COLLISION_BITS) * max_turning_ticks);

    if(right_side < left_side)
    {
      // printf("**********RIGHT\n");
      set_motion(TURN_RIGHT);
      free_space = RIGHT;
    }
    else if(right_side > left_side)
    {
      // printf("**********LEFT\n");
      set_motion(TURN_LEFT);
      free_space = LEFT;
    }
    
    else{
      // printf("**********RANDOM\n");
      set_motion(free_space);
    }
  }
  else
  {
    last_motion_ticks = kilo_ticks;
    straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    set_motion(FORWARD);
  }
  
  
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
                // printf("state transition outside\n");
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

        if(wall_avoidance_start){
          // printf("%d ID:%d start wall avoidance\n", (kilo_ticks, kilo_uid));
          wall_avoidance_procedure(proximity_sensor);
          proximity_sensor = 0;
          wall_avoidance_start = false;
          
        }
        else{
          random_walk();
          finite_state_machine(); 
        }
        
        
}


void printBits(uint8_t num, int num_bits)
{
   for(int bit=0;bit<(sizeof(uint8_t) * num_bits); bit++)
   {
      printf("%i ", num & 0x01);
      num = num >> 1;
   }
}

int main() {
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);
    return 0;
}
