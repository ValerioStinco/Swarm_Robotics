#ifndef GOTO_ALF_H
#define GOTO_ALF_H

namespace argos {
class CSpace;
class CFloorEntity;
class CSimulator;
}

#include <math.h>
#include <time.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>


#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
// #include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>


using namespace argos;

enum command {
    FORWARD = 0,
    LEFT = 1,
    RIGHT = 2,
    STOP = 3
};

class CNavigationALF : public CALF
{

public:

    CNavigationALF();

    virtual ~CNavigationALF();

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

    /** Desired initial position and orientation */
    void GoToWithOrientation(CKilobotEntity &c_kilobot_entity);

    /** Reassign initial position */
    void GreedyAssociation(std::vector<CVector2> actual_pos, std::vector<CVector2> desired_pos);

    /** Print arrived Kilobots */
    void PrintArrivedKilobot();

    /** Print Kilobot Pose */
    void PrintPose(UInt16& unKilobotID, command& cmd);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

    /** Flag to check if kilobot is arrived in its initial desired position */
    std::vector<bool>  v_arrivedInPosition;
    std::vector<bool>  v_arrivedInOrientation;

    //std::vector <int> assignedTargets;
    const double kDistThreshold = 0.0002;   //0.002
    const double kDistPushed = 0.001; //0.0001
    const double kAngleThreshold = 0.17;    //0.17 Radians 9,740283 Degrees

private:


    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea
    {
        CVector2 Center;
        Real Radius;
        CColor Color;
    };

    SVirtualArea m_sClusteringHub;
    std::vector<SVirtualArea> m_TargetAreas;



    
    struct SVirtualPerimeter
    {
        // circular arena variables
        Real circular_arena_radius; // radius for the circular arena
        Real circular_arena_width;  // wall width
        Real circular_arena_height; // wall height
        Real circular_arena_walls;  // number of walls
    };

    SVirtualPerimeter m_WallStructure;


    /* used to initialize the position of each kilobot uniformly */
    std::vector<std::pair<UInt32,UInt32>> index_vector;
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<CRadians> m_vecKilobotsOrientations;
    std::vector<CVector2> m_vecDesInitKilobotPosition;
    std::vector<CRadians> m_vecDesInitKilobotOrientation;
    std::vector<command> m_vecCommandLog;
    
    /** Structure to contain data to evaluate first passage time and
     *  convergence time of the experiment
     *  single item example: <kilo_id, <f_p_t, f_i_t>> 
     * */
    std::map<UInt16,std::pair<UInt32,UInt32>> m_KilobotResults; 

    /* used to store the last message sent to each kilobot */
    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /************************************/
    /*       Experiment variables       */
    /************************************/
  
    /* random number generator */
    CRandom::CRNG* c_rng;

    /* crwlevy exponents */
    Real crw_exponent;
    Real levy_exponent;

    /* output file for data acquisition */
    std::ofstream m_cOutput;

    /* output file name*/
    std::string m_strOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;

    UInt32 m_unFullDiscoveryTime;
    UInt32 m_unFullInformationTime;
    Real m_fFractionWithInformation;
    Real m_fFractionWithDiscovery;
};

#endif

