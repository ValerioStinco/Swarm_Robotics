/**
 * This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots search for a special area. When the robot finds the area, ALF signals him, and he stops moving.
 *
 * Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 */

#include "kilobot_v2.h"

//SERVER
char buf[30];
char bufStore[30];
int bytesReceived=-1;
int num_of_areas=1;
int num_of_kb=25;
int placed = 0;
std::vector<bool> inPlace;
std::vector<bool> filledArea;
std::vector<int> contained;
bool flag1 = 0;
int clientSocket;
int serverSocket; //socket for client-server communication
int target_index;


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
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cout << "Can't create a socket! Quitting" << std::endl;
    }
    int port = 54000;
    filledArea = std::vector<bool>(6,0);
    inPlace = std::vector<bool>(25,0); //vector with 1 corresponding to KBs that stopped moving (they are inside an area)
    contained = std::vector<int>(6,0);
    memset(bufStore, 0, 30);
    std::string ipAddress = "0.0.0.0";
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);
    bind(serverSocket, (sockaddr*)&hint, sizeof(hint));
    listen(serverSocket, SOMAXCONN);
    sockaddr_in client;
    socklen_t clientSize = sizeof(client);
    clientSocket = accept(serverSocket, (sockaddr*)&client, &clientSize);
    char host[NI_MAXHOST];      // Client's remote name
    char service[NI_MAXSERV];   // Service (i.e. port) the client is connect on
    memset(host, 0, NI_MAXHOST); // same as memset(host, 0, NI_MAXHOST);
    memset(service, 0, NI_MAXSERV);
    if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
    {
        std::cout << host << " connected on port " << service << std::endl;
    }
    else
    {
        inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
        std::cout << host << " connected on port " << ntohs(client.sin_port) << std::endl;
    }
    close(serverSocket);
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
    close(clientSocket);
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
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    if (m_vecKilobotStates[unKilobotID]==SEARCHING){
        CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
        Real fDistance = Distance(cKilobotPosition, multiArea[target_index].Center);
        if(fDistance<(multiArea[target_index].Radius*1)){
            m_vecKilobotStates[unKilobotID]=INSIDE_AREA;
            if (inPlace[unKilobotID]==0){
                placed+=1;
                inPlace[unKilobotID]=1;
                std::string buf = std::to_string(placed);
                std::cout<<"sending number of kilobots on target: "<<buf<<std::endl;
                send(clientSocket, buf.c_str(), buf.size() + 1, 0);
            }
        }
    }
    memset(buf, 0, 30);
    bytesReceived = recv(clientSocket, buf, 30, MSG_DONTWAIT);
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
        std::cout<<"String received: "<<bufStore<<std::endl;
        std::cout<<"First char received: "<<((int)bufStore[0]-48)<<std::endl;  //ASCII conversion
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
        if(bufStore[0]==51 && bufStore[1]==51 && bufStore[2]==51){ //333 in ASCII, this detects the starting message and lights up white led
            m_vecKilobotData[unKilobotID].ALL=atoi(bufStore);       //use this when receiving RGB code of area color
        }
        else if (bufStore[0]!=0){
            target_index = ((int)bufStore[0]-48);
            std::string target = multiArea[target_index].RGBcolor;
            m_vecKilobotData[unKilobotID].ALL=atoi(target.c_str());    //use this when receiving area index
            if(m_vecKilobotStates[unKilobotID]==RANDOM_WALKING){
                m_vecKilobotStates[unKilobotID]=SEARCHING;
            }
        }

        UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
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

REGISTER_LOOP_FUNCTIONS(CClusteringALF, "kilobot_v2_loop_function")