#include "kilobot_ALF_dhtf.h"

CALFClientServer::CALFClientServer() :
    m_unDataAcquisitionFrequency(10){
}


CALFClientServer::~CALFClientServer(){
}


void CALFClientServer::Init(TConfigurationNode& t_node) {
    CALF::Init(t_node);
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

    /* Read functioning mode: determine if CLIENT or SERVER */
    TConfigurationNode& tModeNode = GetNode(t_node, "functioning_mode");
    GetNodeAttribute(tModeNode,"mode",MODE);

    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 30);

    /* Opening communication port */
    if(MODE=="SERVER"){
        int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        int port = 54000;
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
        char host[NI_MAXHOST];
        char service[NI_MAXSERV];
        memset(host, 0, NI_MAXHOST);
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
    }
    if(MODE=="CLIENT"){
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        int port = 54000;
        std::string ipAddress = "127.0.0.1";
        sockaddr_in hint;
        hint.sin_family = AF_INET;
        hint.sin_port = htons(port);
        inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);
        connect(serverSocket, (sockaddr*)&hint, sizeof(hint));
    }
}


void CALFClientServer::Reset() {
    m_cOutput.close();
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}


void CALFClientServer::Destroy() {
    m_cOutput.close();
    if (MODE == "SERVER"){
        close(clientSocket);
    }
    if (MODE == "CLIENT"){
        close(serverSocket);
    }
}


void CALFClientServer::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecKilobotData.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    
    /* Compute the number of kilobots on the field*/
    num_of_kbs=0;
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
        num_of_kbs = num_of_kbs+1;
    }

    /* Initialization of kilobots variables */
    request = std::vector<int>(num_of_kbs,0);
    whereis = std::vector<int>(num_of_kbs,-1);
}


void CALFClientServer::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = OUTSIDE_AREAS;
    m_vecLastTimeMessaged[unKilobotID] = -1000;
}


void CALFClientServer::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    TConfigurationNodeIterator itAct;
    
    /* Compute number of areas on the field*/
    num_of_areas=0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct) {
        num_of_areas += 1;
    }

    /* Build the structure with areas data */
    multiArea.resize(num_of_areas);
    int i=0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct, i++){
        std::string s = std::to_string(i);
        std::string var = std::string("Area") + std::string(s);
        TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,var);
        GetNodeAttribute(t_VirtualClusteringHubNode, "position", multiArea[i].Center);
        GetNodeAttribute(t_VirtualClusteringHubNode, "radius", multiArea[i].Radius);
        GetNodeAttribute(t_VirtualClusteringHubNode, "color", multiArea[i].Color);
    }
    for (int ai=0; ai<num_of_areas; ai++){
        multiArea[ai].Completed = false;
    }

    /* Initialization of areas variables */
    contained = std::vector<int>(num_of_areas, 0);
}


void CALFClientServer::GetExperimentVariables(TConfigurationNode& t_tree){
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}


void CALFClientServer::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition = GetKilobotPosition(c_kilobot_entity);

/* Listen for the other ALF communication */
    memset(inputBuffer, 0, 30);
    if (MODE == "SERVER"){
        bytesReceived = recv(clientSocket, inputBuffer, 30, MSG_DONTWAIT);
    }
    if (MODE == "CLIENT"){
        bytesReceived = recv(serverSocket, inputBuffer, 30, MSG_DONTWAIT);
    }
    if ((bytesReceived == -1) || (bytesReceived == 0)){
        //std::cout << "not receiving" << std::endl;
    }
    else 
    {
        /* Save the received string in a vector, for having data available until next message comes */
        for (int i=0; i<30; i++){
            storeBuffer[i] = inputBuffer[i];
        }
        std::cout<<storeBuffer<<std::endl;
    }

/* Build the message for the other ALF */
    std::string outputBuffer = "";
    for (int k=0; k<num_of_areas; k++){

        /* Use this to send the number of kilobots in each area */
        //outputBuffer.append(std::to_string(contained[k]));

        /* Write 1 if the requirements of the area are satisfied for the sender, else write 0 */
        if (multiArea[k].Color.GetRed() == 255){
            if (contained[k] >= 3) {
                outputBuffer.append("1");
            }
            else {
                outputBuffer.append("0");
            }
        }
        else if (multiArea[k].Color.GetBlue() == 255){
            if (contained[k] >= 1) {
                outputBuffer.append("1");
            }
            else {
                outputBuffer.append("0");
            }            
        }
    }
/////////////////////////////////////////////////////////////////////////////////////////////
    /*if (MODE == "SERVER"){
        if (unKilobotID == 0){
            //std::cout<<outputBuffer<<std::endl;
            for (int g=0; g<num_of_areas; g++){
                double r = ((double) rand() / (RAND_MAX));
                //std::cout<<r<<std::endl;
                if ((multiArea[g].Completed == true) && (r < 1)){
                    //std::cout<<"true "<<g<<std::endl;
                    multiArea[g].Completed = false;
                    storeBuffer[g]=48;
                    //outputBuffer.append(std::to_string(g));
                }
            }
        }
    }*/
/////////////////////////////////////////////////////////////////////////////////////////////
    
    /* Send the message to the other ALF*/
    if (unKilobotID == 0){
        if (MODE == "SERVER"){
            send(clientSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
        if (MODE == "CLIENT"){
            send(serverSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }   
    }

    /* Task completeness check */
    for (int j=0; j<num_of_areas; j++){
        if (storeBuffer[j]-48 == 1) {
            if ((multiArea[j].Color.GetBlue() == 255) && (contained[j] >= 1)) {
                multiArea[j].Completed = true;
            }
            if ((multiArea[j].Color.GetRed() == 255) && (contained[j] >= 3)) {
                multiArea[j].Completed = true;
            }
        }
    }

/*State transition*/
    switch (m_vecKilobotStates[unKilobotID]) {
        case OUTSIDE_AREAS : {
            /* Check if the kilobot is entered in a task area */
            for (int i=0;i<num_of_areas;i++){ 
                Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
                if((fDistance < (multiArea[i].Radius*1)) && (multiArea[i].Completed == false) && (GetKilobotLedColor(c_kilobot_entity) != argos::CColor::RED)){
                    m_vecKilobotStates[unKilobotID] = INSIDE_AREA;
                    /* Check the area color to understand the requirements of the task */
                    if (multiArea[i].Color.GetRed() == 255){
                        request[unKilobotID] = 3;
                    }
                    if (multiArea[i].Color.GetBlue() == 255){
                        request[unKilobotID] = 1;
                    }
                    whereis[unKilobotID] = i;
                    contained[i] += 1;
                }
            }
        break;
        }
        case INSIDE_AREA : {
            /* Check if the kilobot has waited too long for colaboratos and it is going away */
            if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::RED){
                m_vecKilobotStates[unKilobotID] = LEAVING;
                contained[whereis[unKilobotID]] -= 1;
            }
            /* Check if the task has been completed */
            else if (multiArea[whereis[unKilobotID]].Completed == true){
                m_vecKilobotStates[unKilobotID] = OUTSIDE_AREAS;
                whereis[unKilobotID] = -1;
                contained[whereis[unKilobotID]] = 0;
            }
        break;
        }
        case LEAVING : {
            Real fDistance = Distance(cKilobotPosition, multiArea[whereis[unKilobotID]].Center);
            /* Check that the robot is a bit far away from the area before returning to OUTSIDE_AREAS, if transition done on the edge it would probably enter again*/
            if (fDistance > (multiArea[whereis[unKilobotID]].Radius*1.2)){
                m_vecKilobotStates[unKilobotID] = OUTSIDE_AREAS;
                whereis[unKilobotID] = -1;
            }
        break;
        }
    }
}


void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    bool bMessageToSend = false;
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg){
        return;
    }
    else{
        /* Compose the message for a kilobot */
        tKilobotMessage.m_sID = unKilobotID;                                //ID of the receiver
        tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID];     //state
        tKilobotMessage.m_sData = request[unKilobotID];                     //requirement of the area where it is
        bMessageToSend = true;
        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
        
    }

    if (bMessageToSend){
        for (int i=0; i<9; ++i) {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;
        for (int i=0; i<3; ++i) {
            if (i == 0){
                tMessage = tKilobotMessage;
            } else{
                tMessage = tEmptyMessage;
            }
            /* Packing the message */
            m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
        }
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }
}


CColor CALFClientServer::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;

    /* Draw ares until they are needed, once that task is completed the corresponding area disappears */
    for (int i=0; i<num_of_areas; i++){
        if (multiArea[i].Completed == false){
            Real fDistance = Distance(vec_position_on_plane,multiArea[i].Center);
                if(fDistance<multiArea[i].Radius){
                    cColor=multiArea[i].Color;
                }
        }
    }
    return cColor;
}


REGISTER_LOOP_FUNCTIONS(CALFClientServer, "kilobot_ALF_dhtf_loop_function")