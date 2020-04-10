/**
 * This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots search for a special area. When the robot finds the area, ALF signals him, and he stops moving.
 *
 * Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 */

#include "kilobot_v1.h"

//CLIENT
char buf[30];
char bufStore[30];
int num_of_areas=1;
int num_of_kb=15;
int bytesReceived=-1;
std::vector<bool> filledArea(6,0);      //is area "i" empty or not
std::vector<bool> inPlace(num_of_kb,0); //vector with 1 corresponding to KBs that stopped moving (they are inside an area)
int placed = 0;                         //number of stopped KBs
std::vector<int> contained(6,0);        //how many KBs the area "i" contains
bool flag1 = 0;
bool flag2 = 0;
int serverSocket = socket(AF_INET, SOCK_STREAM, 0); //socket for client-server communication


CClusteringALF::CClusteringALF() :
    m_unDataAcquisitionFrequency(10){
}


CClusteringALF::~CClusteringALF(){
}


void CClusteringALF::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

//----------------------------------------------OPEN PORT-------------------------------------------------------------------------------------
    int port = 54000;
    std::string ipAddress = "127.0.0.1";
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);
    connect(serverSocket, (sockaddr*)&hint, sizeof(hint));
//--------------------------------------------------------------------------------------------------------------------------------------------
}


void CClusteringALF::Reset() {
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}


void CClusteringALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
    close(serverSocket);
}


void CClusteringALF::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecKilobotData.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
}


void CClusteringALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = RANDOM_WALKING;
    m_vecLastTimeMessaged[unKilobotID] = -1000;
}


void CClusteringALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    TConfigurationNodeIterator itAct;

    for(itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct) {
        num_of_areas=num_of_areas+1;
    }
    multiArea.resize(num_of_areas);
    int i=1;
    for(itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct, i++) {
        std::string s = std::to_string(i);
        std::string var = std::string("Area") + std::string(s);
        TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,var);
        GetNodeAttribute(t_VirtualClusteringHubNode, "position", multiArea[i].Center);
        GetNodeAttribute(t_VirtualClusteringHubNode, "radius", multiArea[i].Radius);
        GetNodeAttribute(t_VirtualClusteringHubNode, "color", multiArea[i].Color);
        multiArea[i].RGBcolor.append(std::to_string(multiArea[i].Color.GetRed()/85));
        multiArea[i].RGBcolor.append(std::to_string(multiArea[i].Color.GetGreen()/85));
        multiArea[i].RGBcolor.append(std::to_string(multiArea[i].Color.GetBlue()/85));
    }
}


void CClusteringALF::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}


void CClusteringALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Update the state of the kilobot (inside or outside the clustering hub)*/

    if (flag2==0){
        std::string buf1 = "333";       //turn on white LED: it means the client swarm started moving
        send(serverSocket, buf1.c_str(), buf1.size() + 1, 0);
        std::cout<<"Sending initial signal "<<buf1<<std::endl;
        flag2 = 1;
    }

    bytesReceived = recv(serverSocket, buf, 30, MSG_DONTWAIT);
    if (bytesReceived == -1){
        //std::cout << "received -1 char" << std::endl;
    }
    else if(bytesReceived == 0){
        //std::cout << "received 0 char" << std::endl;
    }
    else 
    {
        bufStore[0]=buf[0];
        bufStore[1]=buf[1];
        bufStore[2]=buf[2];
        bufStore[3]=buf[3];
        bufStore[4]=buf[4];
        bufStore[5]=buf[5];
        std::cout<<"kilobots on target: "<<bufStore<<std::endl;
    }
    for(int i=1;i<num_of_areas;i++){
        UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
        CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
        Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
        if(fDistance<(multiArea[i].Radius*1)){
            m_vecKilobotStates[unKilobotID]=INSIDE_AREA;
            m_vecKilobotData[unKilobotID].R=(multiArea[i].Color.GetRed())/85;       //get components of area color and divide by 85 to turn from 0-255 to 0-3 range
            m_vecKilobotData[unKilobotID].G=(multiArea[i].Color.GetGreen())/85;     //then assemble in a single number ALL to be transmitted
            m_vecKilobotData[unKilobotID].B=(multiArea[i].Color.GetBlue())/85;
            m_vecKilobotData[unKilobotID].ALL=(m_vecKilobotData[unKilobotID].B)+10*(m_vecKilobotData[unKilobotID].G)+100*(m_vecKilobotData[unKilobotID].R);

//-----------------------------------------SENDING DATA--------------------------------------------------------------------------------------
            /*if (filledArea[i-1]==0){                                     //say which kilobot visit an area for the first time
                std::string buf = "- - Area ";
                buf.append(std::to_string(i));
                buf.append(" discovered by kilobot number ");
                buf.append(std::to_string(unKilobotID));
                send(serverSocket, buf.c_str(), buf.size() + 1, 0);
                filledArea[i-1]=1;
            }*/
            if (inPlace[unKilobotID]==0){                               //say in which area each kilobot stops
                /*std::string buf = "Kilobot ";
                buf.append(std::to_string(unKilobotID));
                buf.append(" placed in area number ");
                buf.append(std::to_string(i));
                send(serverSocket, buf.c_str(), buf.size() + 1, 0);*/
                contained[i-1]+=1;
                placed+=1;
                /*std::string buf1 = std::to_string(placed);
                buf1.append("/");
                buf1.append(std::to_string(num_of_kb));*/
                inPlace[unKilobotID]=1;
                /*send(serverSocket, buf1.c_str(), buf1.size() + 1, 0);
                std::cout<<buf1<<std::endl;*/
            }
//-----------------------------------------------------------------------------------------------------------------------------------------------
            break;
        }
        else{
            m_vecKilobotStates[unKilobotID]=RANDOM_WALKING;
        }
    }

    if (placed==num_of_kb && flag1==0){                                   //say how many kilobot there are in each area
        std::string buf = "Distribution: ";
        for(int k=0;k<num_of_areas-1;k++){
            buf.append(" ");
            buf.append(std::to_string(contained[k]));
        }
        //send(serverSocket, buf.c_str(), buf.size() + 1, 0);
        std::cout<<buf<<std::endl; //list of number of KBs in each area
        int actual_max=0;
        int max_index=0;
        for (int t=0;t<num_of_areas-1;t++){
            if (contained[t]>actual_max){
                actual_max=contained[t];
                max_index=t+1;
            }
        }
        std::string maxs = std::to_string(max_index);
        //send(serverSocket, maxs.c_str(), maxs.size() + 1, 0);
        std::cout<<"Most populated area: "<<max_index<<std::endl;
        //std::cout<<multiArea[max_index].Center<<std::endl;
        //buf = multiArea[max_index].RGBcolor;                 //use this to send directly the RGB color
        buf = maxs;                                            //use this to send the number of the target area
        std::cout<<"Sending index "<<buf<<std::endl;
        send(serverSocket, buf.c_str(), buf.size() + 1, 0);            //send the RGB color of the most populated area in client experiment
        flag1=1;
    }
}


void CClusteringALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Flag for existance of message to send*/
    bool bMessageToSend=false;
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    else{
        /*  Prepare the inividual kilobot's message */
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID];
        tKilobotMessage.m_sData = (int)m_vecKilobotData[unKilobotID].ALL;
        /*  Set the message sending flag to True */
        bMessageToSend=true;
        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    }

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    if(bMessageToSend){
        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }
        // Prepare an empty ARK-type message to fill the gap in the full kilobot message
        tEmptyMessage.m_sID=1023;
        tEmptyMessage.m_sType=0;
        tEmptyMessage.m_sData=0;
        // Fill the kilobot message by the ARK-type messages
        for (int i = 0; i < 3; ++i) {
            if( i == 0){
                tMessage = tKilobotMessage;
            } else{
                tMessage = tEmptyMessage;
            }
            m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
        }
        /* Sending the message */
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }
}


CColor CClusteringALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    int i;
    for (i=1; i<num_of_areas; i++){
    Real fDistance = Distance(vec_position_on_plane,multiArea[i].Center);
        if(fDistance<multiArea[i].Radius){
            cColor=multiArea[i].Color;
        }
    }
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CClusteringALF, "kilobot_v1_loop_function")