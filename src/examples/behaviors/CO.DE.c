/*
 * Kilobot control software for a decision making simulation over different resources.
 *
 * Explanation about the code:
 * The kilobots are expected to exploit a set of resources displaced over the arena.
 * The kilobots are equipped (this is a simulated sensor) with a sensor that allows them to sense the region of the arena
 * directly beneath.
 * The kilobots can only estimate the overall resources statuses and to do so they use and exponential moving average
 * computed over time and on multiple scans.
 *
 * The kilobots implement a stochastic strategy for exploiting the resources and switch between three different statuses:
 * - uncommitted - exploring and not exploiting any resource
 * - committed and looking for the resource - actively looking for a region cotaining the resource
 * - committed and working - working over the are and exploiting the resource. In this phase the kilobot stands still
 *
 * The kilobots have their leds set:
 * - off if uncommitted
 * - red if committed to resource 1 (either working or not)
 * - green if committed to resource 2 (either working or not)
 * - blued if committed to resource 3 (either working or not)
 *
 * NOTE increase all ticks by a factor of 10 when dealing with real simulations
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#include "kilolib.h"
// do not change the order of includes here
// used for debug with ARGoS simulator
#include "complexity.h"
#include "distribution_functions.c"
#include "message_t_list.c"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

// define the resources to be expected in the current simulation
#define RESOURCES_SIZE 2
#ifndef M_PI
#define M_PI 3.14159
#endif
/*-------------------------------------------------------------------*/
/* Motion Variables                                                  */
/*-------------------------------------------------------------------*/

/* enum for different motion types */
typedef enum {
              FORWARD = 0,
              TURN_LEFT = 1,
              TURN_RIGHT = 2,
              STOP = 3,
} motion_t;

/* current motion type */
motion_t current_motion_type = FORWARD;

/* counters for motion, turning and random_walk */
const float std_motion_steps = 10*31; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2; // 2 is brownian like motion (alpha)
const float  crw_exponent = 0.0; // higher more straight (rho)
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 80; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
uint32_t straight_ticks = 0; // keep count of ticks of going straight
const uint16_t max_straight_ticks = 2*31;
uint32_t last_motion_ticks = 0;

// the kb is biased toward the center when close to the border
float rotation_to_center = 0; // if not 0 rotate toward the center (use to avoid being stuck)
uint8_t rotating = 0; // variable used to cope with wall avoidance (arena borders)

/*-------------------------------------------------------------------*/
/* Smart Arena Variables                                             */
/*-------------------------------------------------------------------*/

/* enum for the robot states/position w.r.t. the smart arena */
typedef enum {
              OUTSIDE_AREA=255,
              INSIDE_AREA_0=0,
              INSIDE_AREA_1=1,
              INSIDE_AREA_2=2,
              INSIDE_AREA_01=3,
              INSIDE_AREA_02=6,
              INSIDE_AREA_12=7,
              INSIDE_AREA_012=21,
} arena_t;

/* enum for keeping trace of internal kilobots decision related states */
typedef enum {
              NOT_COMMITTED=255,
              COMMITTED_AREA_0=0,
              COMMITTED_AREA_1=1,
              COMMITTED_AREA_2=2,
              QUORUM_AREA_0=3, // only if quorum sensing is enabled
              QUORUM_AREA_1=4, // only if quorum sensing is enabled
              QUORUM_AREA_2=5, // only if quorum sensing is enabled
} decision_t;

/* current state */
arena_t current_arena_state = OUTSIDE_AREA;
decision_t current_decision_state = NOT_COMMITTED;

/* quorum sensing variables */
float quorum_threshold = 0;
uint8_t real_quorum[RESOURCES_SIZE]  = {255,255,255}; // keep this as > 127

/* variable to signal internal computation error */
uint8_t internal_error = 0;

/* Exponential Moving Average alpha */
const float ema_alpha = 0.1;
/* Umin threshold for the kb in 255/31 splice */
const uint8_t umin = 153; //0.6
/* Umax threshold for utility scaling between umin and umax */
uint8_t umax = 255; // do not change!

/* Variables for Smart Arena messages */
uint8_t resources_pops[RESOURCES_SIZE]; // keep local knowledge about resources

/*-------------------------------------------------------------------*/
/* Decision Making                                                   */
/*-------------------------------------------------------------------*/
/* system time scale -- a control variable       */
/* used to increase or decrease the system speed */
const float tau = 1;

/* processes variables */
const float h = 0.1111111; // determines the spontaneous (i.e. based on own information) processes weight
const float k = 0.8888889; // determines the interactive (i.e. kilobot-kilobot) processes weight

/* explore for a bit, estimate the pop and then take a decision */
/* the time of the kilobot is 31 ticks per second, no matter what time you set in ARGOS */
uint32_t last_decision_ticks = 0; /* when last decision was taken */
const uint32_t exploration_ticks = 5*31; /* take a decision only after exploring the environment */

/*-------------------------------------------------------------------*/
/* Communication                                                     */
/*-------------------------------------------------------------------*/
/* flag for message sent */
uint8_t sent_message = 1;

/* turn this flag on if there is a valid message to send */
uint8_t to_send_message = false;

/* for kb messages out */
message_t interactive_message;
node_t* last_rebroadcasted;

/* avoid concurrent accesses to the list of interactive messages */
char to_parse_semaphore;
message_t to_parse_message;

/* for broacasts */
#ifdef ARGOS_simulator_BUILD
// in ARGoS we do not need particular communication protocol, we
// do not care about collission and message propagation
const uint32_t broadcast_ticks = 0*31; // one message every broadcast_ticks (not the same as below with ARK!)
uint32_t last_broadcast_ticks = 0; // when last broadcast occurred
#else
// with real kilobots we adopt a different message propagation strategy to avoid
// collision and medium overload
char release_the_broadcast = 0; // if true, start broadcasting until false
uint32_t last_release_time = 0; // used to restore the state of the kilobot after freezing it for broadcast
char first_time_after_release = 0;
#endif

/* messages are valid for valid_until ticks */
const uint32_t valid_until = 15*31;

/* buffer for communications */
/* used both for flooding protocol and for dm */
node_t *b_head = NULL;
/* list size */
uint16_t list_size;
/* count messages from smart arena */
uint8_t messages_count;

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type) {
  if(current_motion_type != new_motion_type ) {
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
/* Merge received information about the population of the area on    */
/* which the kb is on. Kilobots can only perceive locally and try to */
/* estimate the population of a resource                             */
/*-------------------------------------------------------------------*/

void exponential_average(uint8_t resource_id, uint8_t resource_pop) {
  // update by using exponential moving averagae to update estimated population
  resources_pops[resource_id] = (uint8_t)round(((float)resource_pop*(ema_alpha)) + ((float)resources_pops[resource_id]*(1.0-ema_alpha)));
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/* as in the complexity_ALF.cpp there are 3 kilobots messages per    */
/* message with 3 different kilobots ids.                            */
/*                                                                   */
/* type 0 for arena / type 1 for kbs interactive msgs                */
/*                                                                   */
/* A message structure is 12 bytes in length and is composed of      */
/* three parts: the payload (9 bytes), the message type (1 byte),    */
/* and a CRC (2 bytes).                                              */
/*-------------------------------------------------------------------*/

/* see next function for specification of what is done here */
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_position) {
  // update message count
  messages_count++;

  // index of first element in the data
  uint8_t shift = kb_position*3;

  // get arena state by resource (at max 3 resources allowed in the simulation)
  uint8_t ut_a = (data[0+shift] &0x01) << 4 | (data[1+shift] &0xF0) >> 4;
  uint8_t ut_b = (data[1+shift] &0x0F) << 1 | (data[2+shift] &0x80) >> 7;
  uint8_t ut_c = (data[2+shift] &0x7C) >> 2;
  /* if(kilo_uid == 0) { */
  /*   printf("data0 %d data1 %d data2 %d\n", data[0+shift], data[1+shift], data[2+shift]); */
  /*   printf("ut_a %d ut_b %d ut_c %d\n", ut_a, ut_b, ut_c); */
  /*   fflush(stdout); */
  /* } */

  if(ut_a+ut_b+ut_c == 0) {
    // set this up if over no resource
    current_arena_state = 255;
  } else {
    current_arena_state = 0;
    if(ut_c) {
      // either 0 (on a) or 2 (on c)
      current_arena_state = 2;
    }
    if(ut_b) {
      // either 0 (on a) or 1 (on b) or 7 (on b and c)
      current_arena_state = current_arena_state*3+1;
    }
    if(ut_a) {
      // either 0 (on a) 3 (on a and b) 6 (on a and c) 21 (on a and b and c)
      current_arena_state = current_arena_state*3;
    }
  }

  // store received utility
  // 31 slices of means every 8
  if(ut_a) {
    uint8_t ut = ceil(ut_a*8.2258);
    exponential_average(0, ut);
  }
  if(ut_b) {
    uint8_t ut = ceil(ut_b*8.2258);
    exponential_average(1, ut);
  }
  if(ut_c) {
    uint8_t ut = ceil(ut_c*8.2258);
    exponential_average(2, ut);
  }

  // get rotation toward the center (if far from center)
  // avoid colliding with the wall
  uint8_t rotation_slice = data[2+shift] &0x03;
  if(rotation_slice == 3) {
    rotation_to_center = -M_PI/2;
  } else {
    rotation_to_center = (float)rotation_slice*M_PI/2;
  }
}

void parse_interactive_message() {
  /* ----------------------------------*/
  /* KB interactive message            */
  /* ----------------------------------*/
  char to_consider = 0;
  // store the message in the buffer for flooding and dm
  // check if it has been parsed before and if enough time is passed, update it
  // avoid resending same messages over and over again
  int16_t m_pos = mtl_is_message_present(b_head, to_parse_message);
  node_t* t_node;
  if(m_pos < 0) {
    // message is new, store it
    t_node = malloc(sizeof(node_t));
    t_node->msg = to_parse_message;
    
    // green light for the rx callback
    to_parse_semaphore = 0;

    t_node->time_stamp=kilo_ticks;
    t_node->been_rebroadcasted=false;
    if(b_head == NULL) {
      *b_head = *t_node;
    } else {
      mtl_push_back(b_head, t_node);
    }
    // consider this message information for local updates
    to_consider = 1;
  } else {
    // get the pointer for update
    t_node = mtl_get_at(b_head, m_pos);
    // if old enough (~1 sec)
    if(t_node->time_stamp < kilo_ticks - 31) {
      t_node->msg = to_parse_message; // update content
  
      // green light for the rx callback
      to_parse_semaphore = 0;

      t_node->been_rebroadcasted = 0; // signal to rebroadcast
      t_node->time_stamp = kilo_ticks; // update reception time
      // consider this message information for local updates
      to_consider = 1;
    }
  }

  if(to_consider) {
    // check received message and merge info and ema
    if(t_node->msg.data[3] > 0) {
      exponential_average(0, t_node->msg.data[3]);
    }
    if(t_node->msg.data[4] > 0) {
      exponential_average(1, t_node->msg.data[4]);
    }
    if(t_node->msg.data[5] > 0) {
      exponential_average(2, t_node->msg.data[5]);
    }
    // update umax
    umax = (uint8_t)round(((float)t_node->msg.data[8]*(ema_alpha)) + ((float)umax*(1.0-ema_alpha)));
  }
}

void message_rx(message_t *msg, distance_measurement_t *d) {
  // if type 0 is either from ARGoS or ARK
  if(msg->type==0) {
    /* ----------------------------------*/
    /* smart arena message               */
    /* ----------------------------------*/

    /* see README.md to understand about ARK messaging */
    /* data has 3x24 bits divided as                   */
    /*  data[0]   data[1]   data[2]                    */
    /* xxxx xxxa aaaa bbbb bccc ccyy                   */
    /* x(7) bits used for kilobot id                  */
    /* a(5) bits used for resource a utility           */
    /* b(5) bits used for resource b utility           */
    /* c(5) bits used for resource c utility           */
    /* y(2) bits used for turing angle                 */

    /* How to interpret the data received?             */
    /* If no resource a,b,c utility is received then   */
    /* means that the kb is on empty space             */
    /* On the opposite, if a resource utility is there */
    /* means that the kb is on resource space          */
    /* The turning angle is divided in 4 equal slices  */
    /* pi/4 to 3/4pi - 3/4pi to 5/4pi - 5/4pi to 7/4pi */

    // ids are first 10 bits
    uint16_t id1 = msg->data[0] >> 1;
    uint16_t id2 = msg->data[3] >> 1;
    uint16_t id3 = msg->data[6] >> 1;

    if(id1 == kilo_uid) {
      parse_smart_arena_message(msg->data, 0);
    } else if(id2 == kilo_uid) {
      parse_smart_arena_message(msg->data, 1);
    } else if (id3 == kilo_uid) {
      parse_smart_arena_message(msg->data, 2);
    }
  }
  else if(msg->type==1 && !to_parse_semaphore) {
    /* get id (always firt byte when coming from another kb) */
    uint8_t id = msg->data[0];
    // check that is a valid crc and another kb
    if(id!=kilo_uid){
      // store the message for later parsing to avoid the rx to interfer with the loop
      to_parse_message = *msg;
      // signal to the loop that we have a message to parse 
      to_parse_semaphore = 1;
    }
  }
  else if(msg->type==120) {
    // kilobot signal id message (only used in ARK to avoid id assignment)
    // note that here the original ARK messages are used hence the id is assigned in the
    // first two bytes of the message
    uint16_t id = (msg->data[0] << 8) | msg->data[1];
    if (id == kilo_uid) {
      set_color(RGB(0,0,3));
    } else {
      set_color(RGB(3,0,0));
    }
  }
}


/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
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


/*-------------------------------------------------------------------*/
/* successful transmission callback                                  */
/*-------------------------------------------------------------------*/

void message_tx_success() {
  sent_message = 1;
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

    /* if(kilo_uid == 0) { */
    /*   printf("res 1 %d - 2 %d - 3 %d \n", resources_pops[0], resources_pops[1], resources_pops[2]); */
    /*   printf("res considered %d %d \n", random_resource, recruiter_state); */
    /*   printf("in commitment and recruitment: %d, %d, %d\n", commitment, recruitment, extraction); */
    /* fflush(stdout); */
    /* } */
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


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/

void random_walk(){
  /* if the arena signals a rotation, then rotate toward the center immediately */
  if(rotation_to_center != 0 && !rotating) {
    if(rotation_to_center > 0) {
      set_motion(TURN_LEFT);
    } else {
      set_motion(TURN_RIGHT);
    }
    // when too close to the border bias toward center
    float angle = abs(rotation_to_center);

    /* compute turning time */
    turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
    straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    return;
  }

  /* else keep on with normal random walk */
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
      float angle = 0; // rotation angle

      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      if (rand_soft() % 2) {
        set_motion(TURN_LEFT);
      } else {
        set_motion(TURN_RIGHT);
      }
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
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
  /* Initialise LED and motors */
  set_color(RGB(0,0,0));
  /* Initialise motion variables */
  set_motion(FORWARD);
  /* Initialize quorum vars and resources */
  uint8_t i;
  for(i=0; i<RESOURCES_SIZE; i++) {
    resources_pops[i] = 0;
    real_quorum[i] = 255; // 255 means we still don't know which mechanism to use
  }
  // reset seaphore
  to_parse_semaphore = 0;
}

/*-------------------------------------------------------------------*/
/* Quorum Sensing                                                    */
/*-------------------------------------------------------------------*/
void quorum_sensing() {
  /* check quorum every step if in quorum state */
  if(quorum_threshold > 0 &&
     current_decision_state > 2 &&
     current_decision_state != NOT_COMMITTED) {

    uint8_t neighbors = 0; // the total number of agents sensed
    uint8_t friends = 0; // the number of agents with same quorum state

    /* 
    * we here implement both real quorum (i.e., ARK perceptions broadcasted to the kilobots see message_rx)
    * and perceived quorum (i.e., as perceived by the kilobots)
    * If any of the value in the array is 255, then the latter is implemented
    */
    if(real_quorum[0]<255 && real_quorum[1]<255 && real_quorum[2]<255) { 
      neighbors = real_quorum[0] + real_quorum[1] + real_quorum[2];
      friends = real_quorum[current_decision_state-3];
    } else {
      // avoid considering same neighbor
      char parsed[255] = {0};

      node_t* temp = b_head;
      // cycle over all messages in the buffer
      while(temp) {
        // if already considered skip
        if(!parsed[temp->msg.data[0]]) {
          // if committed or quorum to same resource then we have a friend
          // the following works because 255 for current_decision_state is not an option
          if(temp->msg.data[1] == current_decision_state ||
            temp->msg.data[1]-3 == current_decision_state ||
            temp->msg.data[1]+3 == current_decision_state) {
            friends++;
          }
          // increment the number of neighbors
          neighbors++;
          // set has parsed
          parsed[temp->msg.data[0]] = 1;
        }
        // increment temp
        temp = temp->next;
      }
    }
    
    // compute quorum and eventually switch to committed
    if(neighbors > 0 && ((float)neighbors*quorum_threshold) <= friends) {
      current_decision_state = current_decision_state-3;
    }
  }
}

/* Parse the decision and act accordingly */
void update_led_status() {
    if(current_decision_state==0) 
      set_color(RGB(0,0,0));
    else if(current_decision_state==1) 
      set_color(RGB(3,0,0));
    else if(current_decision_state==2) 
      set_color(RGB(0,3,0));
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
    interactive_message.data[2] = current_arena_state;
    // share my resource pop for all resources
    uint8_t res_index;
    for(res_index=0; res_index<RESOURCES_SIZE; res_index++) {
      interactive_message.data[3+res_index] = resources_pops[res_index];
    }

    // last byte used for umax
    interactive_message.data[8] = umax;

    // fill up the crc
    interactive_message.crc = message_crc(&interactive_message);

    // tell that we have a msg to send
    to_send_message = true;
    // avoid rebroadcast to overwrite prev message
    sent_message = 0;
}

/* Ask the kilobot to get a message to rebroadcast */
void get_message_for_rebroadcast() {
  // random prob of ~20% of sending own message again
  // this cope with inefficient communication on the real kilobots
  if(rand_soft() < 50) {
    send_own_state();
    return;
  }

  // avoid dangling pointer to freed memory
  uint16_t last_rebroadcasted_time = last_rebroadcasted->time_stamp;

  // clean list (remove outdated messages)
  list_size = mtl_clean_old(&b_head, kilo_ticks-valid_until);

  // check if we have last rebroadcasted and in case if it has been deleted by the clean old
  if(!last_rebroadcasted || last_rebroadcasted_time < kilo_ticks-valid_until) {
    last_rebroadcasted = mtl_get_first_not_rebroadcasted(b_head);
  } else {
    last_rebroadcasted = mtl_get_first_not_rebroadcasted(last_rebroadcasted);  
  }

  // if there is a valid message then set it up for rebroadcast
  if(last_rebroadcasted) {
    // set it up for rebroadcast
    interactive_message = last_rebroadcasted->msg;
    // update the rebroadcasted status in the message
    last_rebroadcasted->been_rebroadcasted = true;
    // tell that we have a msg to send
    to_send_message = true;
    // avoid rebroadcast to overwrite prev message
    sent_message = 0;

  }
}

void update_umax(decision_t temp_decision) {
  // if I am working and was not on same area before
  if(current_decision_state < 3 && current_decision_state != temp_decision) {
    // take the maximum population
    uint8_t t_max = resources_pops[0];
    if(t_max < resources_pops[1]) t_max = resources_pops[1];
    if(t_max < resources_pops[2]) t_max = resources_pops[2];

    // update umax
    umax = round(t_max*ema_alpha + umax*(1.0-ema_alpha));
  if(kilo_uid == 0)
    printf("umax %d \n", umax);
  }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
  // parse the received interactive message if any 
  if(to_parse_semaphore) {
    parse_interactive_message();
  }
/*
 * if ARK is signaling to rebroacast, then the kilobots stops what they are doing and start rebroadcasting
 * until ARK signals otherwise (message type 2 and 3 are used for these operations)
 */
  if(release_the_broadcast) {
    if(first_time_after_release) {
     send_own_state();
     first_time_after_release = 0;
    } else if(sent_message) {
      get_message_for_rebroadcast();
    }

    // stop moving
    set_motion(STOP);
    update_led_status();

    // do not do anything else (freeze)
    return;
  }

  if(first_time_after_release) {
    first_time_after_release = 0;

    // stop sending messages
    to_send_message = 0;

    // temp var for umax update
    uint8_t temp_decision = current_decision_state;

    // it is time to take the next decision
    take_decision();
    quorum_sensing();
    update_led_status();
    update_umax(temp_decision);
 }
  /* always random walk, never stop even when exploiting */
  random_walk();
}

int main() {
  kilo_init();
  // register message reception callback
  kilo_message_rx = message_rx;
  // register message transmission callback
  kilo_message_tx = message_tx;
  // register tranmsission success callback
  kilo_message_tx_success = message_tx_success;
  kilo_start(setup, loop);

  return 0;
}
