/* Kilobot control software for the simple ALF experment : clustering
 * author: Mohamed Salaheddine Talamali (University of Sheffield) mstalamali1@sheffield.ac.uk
 */

#include "kilolib.h"
#include <math.h>
#include "distribution_functions.c"
// ****************************************
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#if REAL
#define DEBUG
#include <stdarg.h>
#include "debug.h"
#include <avr/eeprom.h>
#else
#include <inttypes.h>
#endif
/* Enum for different motion types */
typedef enum {
    FORWARD = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    STOP = 3,
} motion_t;

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;


/* current motion type */
motion_t current_motion_type = STOP;

/* current state */
uint8_t received_command = FORWARD;



/*Max number of ticks*/
//const double max_time = 9288; //10^3
/* counters for motion, turning and random_walk */
const double std_motion_steps = 5*16;
double levy_exponent = 1.2; // 2 is brownian like motion
double crw_exponent = 0.8; // go straight often
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 320; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint16_t straight_ticks = 0; // keep count of ticks of going straight


uint32_t last_motion_ticks = 0;
/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;
const uint16_t max_info_ticks = 7 * 16;
uint32_t last_info_ticks = 0;

void my_printf(const char *fmt, ...)
{
#if REAL
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);

  va_end(args);
#endif
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type)
{
  if (current_motion_type != new_motion_type)
  {
    switch (new_motion_type)
    {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left, kilo_straight_right);
      // set_motors(1, 1);
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left, 0);
      // set_motors(1, 0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0, kilo_turn_right);
      // set_motors(0, 1);
      break;
    case STOP:
      set_motors(0, 0);
      break;
      
    default:
      set_motors(0, 0);
    }
    current_motion_type = new_motion_type;
  }

}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{

  /* Initialise LED and motors */
  set_motors(0, 0);
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
  
  /* Initialise motion variables */
  set_motion(STOP);

}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/* as in the crwlevy_ALF.cpp there are 3 kilobots messages per    */
/* message with 3 different kilobots ids.                            */
/*                                                                   */
/* type 0 for ark / type 1 for kbs interactive msgs                  */
/* type 255 is a system message from ARK for levy and crw exponents  */
/* and initial desired position                                      */
/* type 254 is a system message from ARK to send commands to reach   */
/* initial desired position                                          */
/*                                                                   */
/* A message structure is 12 bytes in length and is composed of      */
/* three parts: the payload (9 bytes), the message type (1 byte),    */
/* and a CRC (2 bytes).                                              */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d) {
  // printf("%" PRIu32 "\n", msg->type);

  uint8_t cur_distance = estimate_distance(d);
  if (cur_distance > 100) //100 mm  
  {
    return;
  }

  if (msg->type == 254)
  {
    received_command = msg->data[0];
  }

}

// /*-------------------------------------------------------------------*/
// /* Function implementing the crwlevy random walk                     */
// /*-------------------------------------------------------------------*/
// void random_walk()
// {
// 	switch (current_motion_type)
// 	{
// 	case TURN_LEFT:
// 	case TURN_RIGHT:
// 		if (kilo_ticks > last_motion_ticks + turning_ticks)
// 		{
// 		/* start moving forward */
// 		last_motion_ticks = kilo_ticks;
// 		set_motion(FORWARD);
// 		}
// 		break;
// 	case FORWARD:
// 		if (kilo_ticks > last_motion_ticks + straight_ticks)
// 		{
//       /* perform a random turn */
//       last_motion_ticks = kilo_ticks;
//       if (rand_soft() % 2)
//       {
//         set_motion(TURN_LEFT);
//       }
//       else
//       {
//         set_motion(TURN_RIGHT);
//       }
//       double angle = 0;
//       if (crw_exponent == 0)
//       {

//         angle = (uniform_distribution(0, (M_PI)));
//         // my_printf("%" PRIu32 "\n", turning_ticks);
//         // my_printf("%u" "\n", rand());
//       }
//       else
//       {
//         angle = fabs(wrapped_cauchy_ppf(crw_exponent));
//       }
//       turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
//       straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
//       // my_printf("%u" "\n", straight_ticks);
// 		}
// 		break;

// 	case STOP:
// 	default:
// 		set_motion(FORWARD);
// 	} 
// }




/*-------------------------------------------------------------------*/
/* Reset the experiment at start                                     */
/*-------------------------------------------------------------------*/
void check_reset()
{
  if (kilo_ticks == 0) // NOT THE RIGHT TEST, kilo_tick doesnt reinitiate?
  {
    setup();
  }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
/**
 * Hints: il current_init_state probabilmente non ti serve
 **/
void loop()
{
  check_reset();

  // printf("Stato NON arrivato\n");
  // printf("%" PRIu32 "\n", received_command);

  // Randomness in the movement to avoid collision
  if (!(rand()%30) && received_command == FORWARD)
  {
    received_command = TURN_RIGHT;
  }

  set_motion(received_command);
  if(received_command == TURN_LEFT || received_command == TURN_RIGHT)
  {
    set_color(RGB(0, 3, 0));
    delay(500);
    set_color(RGB(3, 0, 0));
    delay(500);

  }
}
  

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    kilo_init();
    // register message reception callback
    kilo_message_rx = message_rx;

     
    kilo_start(setup, loop);

    return 0;
}

