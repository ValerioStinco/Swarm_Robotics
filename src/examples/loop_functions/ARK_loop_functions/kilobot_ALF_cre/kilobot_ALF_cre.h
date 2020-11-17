#ifndef CLUSTRING_ALF_H
#define CLUSTRING_ALF_H

namespace argos {
class CSpace;
class CFloorEntity;
class CSimulator;
}

#include <math.h>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <numeric>
#include <array>


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>


#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>


using namespace argos;


class CALFClientServer : public CALF
{

public:

    CALFClientServer();

    virtual ~CALFClientServer();

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void Destroy();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Experiment configuration methods (From .argos files) */

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode& t_tree);

    /** Virtual environment visualization updating */


    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);


    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

private:
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea             //parameters of the circular areas
    {
        CVector2 Center;
        Real Radius;
        CColor Color;
        bool Completed;             //set to "true" after the task is completed
    };
    std::vector<SVirtualArea> multiArea;

    typedef enum                    //states of the kilobots
    {
        OUTSIDE_AREAS=0,
        INSIDE_AREA=1,
        LEAVING=2,  
    } SRobotState;

    struct FloorColorData           //contains components of area color
    {
        int R;
        int G;
        int B;
    };
    std::vector<FloorColorData> m_vecKilobotData;

    struct TransmittingKilobot             //parameters of the circular areas
    {
        int xCoord;
        int yCoord;
        int commit;
    };
    std::vector<TransmittingKilobot> multiTransmittingKilobot;

    std::string MODE;
    std::string IP_ADDR;            //ip address where to connect
    unsigned int random_seed;
    int desired_num_of_areas;
    float reactivation_rate;
    float communication_range;
    char inputBuffer[2000];           //array containing the message received from the socket
    std::string outputBuffer;          //array  containing the message to send
    char storeBuffer[2000];           //array where to store input message to keep it available
    int bytesReceived;              //length of received string
    int serverSocket;
    int clientSocket;
    int num_of_areas;               //number of clustering areas
    int lenMultiArea;
    int num_of_kbs;                 //number of kilobots on the field
    int arena_update_counter;
    bool initializing;
    bool flag;

    /*vectors as long as the number of kilobots*/
    std::vector<int> actual_orientation;   //vector containing real time orientations
    std::vector<int> command;              // contains informations about actual semiplan direction where the robot tends to go
    std::vector<int> visible_green;
    std::vector<int> visible_red;

    /*vectors as long as the number of areas*/
    std::vector<int> contained;     //how many KBs the area "i" contains
    
    std::vector<SRobotState> m_vecKilobotStates_ALF;        //kb state from ARK poin of view
    std::vector<SRobotState> m_vecKilobotStates_transmit;   //state to be transmitted to the robot
    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /************************************/
    /*       Experiment variables       */
    /************************************/

    /* output file for data acquizition */
    std::ofstream m_cOutput;

    /* output file name*/
    std::string m_strOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;
};

#endif