#include "kilobot_ALF_cre.h"

CALFClientServer::CALFClientServer() :
    m_unDataAcquisitionFrequency(10){
}


CALFClientServer::~CALFClientServer(){
}


void CALFClientServer::Init(TConfigurationNode& t_node) {
    CALF::Init(t_node);
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

    /* Read parameters */
    TConfigurationNode& tModeNode = GetNode(t_node, "extra_parameters");
    GetNodeAttribute(tModeNode,"mode",MODE);
    GetNodeAttribute(tModeNode,"desired_num_of_areas",desired_num_of_areas);
    GetNodeAttribute(tModeNode,"hard_tasks",hard_tasks);
    GetNodeAttribute(tModeNode,"reactivation_rate",reactivation_rate);

    /* Randomly select the desired number of tasks between the available ones, set color and communicate them to the client */
    int t=0;
    if (MODE=="CLIENT"){
        outputBuffer="I";
        t=0;
        while(num_of_areas>desired_num_of_areas){
            double r = ((double) rand() / (RAND_MAX));
            if (r<0.2){
                for (int b = t; b < num_of_areas; b++){
                    multiArea[b] = multiArea[b + 1];
                }
                num_of_areas--;
                int n = t+97;
                char A = static_cast<char>(n);
                std::string s(1, A);
                outputBuffer.append(s);
            }
            t++;
            if(t==num_of_areas){
                t=0;
            }
        }
    }

    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 130);
    arena_update_counter = 500;
    flag=0;
    initializing = true;
    outputBuffer = "";

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
    m_vecKilobotStates_ALF.resize(m_tKilobotEntities.size());
    m_vecKilobotStates_transmit.resize(m_tKilobotEntities.size());
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
    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
    m_vecKilobotStates_transmit[unKilobotID] = OUTSIDE_AREAS;
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
        //GetNodeAttribute(t_VirtualClusteringHubNode, "color", multiArea[i].Color);    // use this to read areas color from .argos
    }
    /* Blue set as default color, then some of the areas turn red */
    for (int ai=0; ai<num_of_areas; ai++){
        multiArea[ai].Completed = false;
        multiArea[ai].Color = argos::CColor::GREEN;
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
    memset(inputBuffer, 0, 130);
    if (MODE == "SERVER"){
        bytesReceived = recv(clientSocket, inputBuffer, 130, MSG_DONTWAIT);
    }
    if (MODE == "CLIENT"){
        bytesReceived = recv(serverSocket, inputBuffer, 130, MSG_DONTWAIT);
    }
    if ((bytesReceived == -1) || (bytesReceived == 0)){
        //std::cout << "not receiving" << std::endl;
    }
    else 
    {
        /* Save the received string in a vector, for having data available until next message comes */
        for (int i=0; i<130; i++){
            storeBuffer[i] = inputBuffer[i];
        }
        std::cout<<storeBuffer<<std::endl;
    }

    /* --------- SERVER --------- */
    if (MODE=="SERVER"){
        /* Initialize the tasks selected by the server */
        if ((storeBuffer[0]==73)&&(initializing==true)){    //73 is the ASCII binary for "I"
            for (int a=1; a<130; a++){
                int n = storeBuffer[a]-97;
                if (n>=0){
                    for (int b = n; b < num_of_areas; b++){
                        multiArea[b] = multiArea[b + 1];
                    }
                    num_of_areas--;
                }
            }
            initializing=false;
        }

        /* Align to server arena */
        if ((storeBuffer[0]==65)&&(initializing==false)){ //65 is the ASCII binary for "A"
            //std::cout<<storeBuffer<<std::endl;
            for (int a=0; a<num_of_areas; a++){
                if (storeBuffer[a+1]-48 == 0) {
                    multiArea[a].Completed = false;
                }
                else{
                    multiArea[a].Completed = true;
                }
            }
        }
    }


    /* --------- CLIENT --------- */
    if (MODE=="CLIENT"){
        /* Task completeness check 
        if (storeBuffer[0]==84){ //84 is the ASCII binary for "T"
            for (int j=0; j<num_of_areas; j++){
                if (storeBuffer[j+1]-48 == 2) {
                    if ((multiArea[j].Color.GetRed() == 255) && (contained[j] >= 6)) {
                        multiArea[j].Completed = true;
                        std::cout<<"red-red task completed"<<std::endl;
                    }
                    if ((multiArea[j].Color.GetBlue() == 255) && (contained[j] >= 2)) {
                        multiArea[j].Completed = true;
                        std::cout<<"blue-red task completed"<<std::endl;
                    }
                }
                if (storeBuffer[j+1]-48 == 1) {
                    if ((multiArea[j].Color.GetRed() == 255) && (contained[j] >= 6)) {
                        multiArea[j].Completed = true;
                        std::cout<<"red-blue task completed"<<std::endl;
                    }
                    if ((multiArea[j].Color.GetBlue() == 255) && (contained[j] >= 2)) {
                        multiArea[j].Completed = true;
                        std::cout<<"blue-blue task completed"<<std::endl;
                    }
                }
            }
        }*/
        /* Reactivate tasks already comlpeted (server routine) */
        if (arena_update_counter == 0){
            for (int a=0; a<num_of_areas; a++){
                if (multiArea[a].Completed == true){
                    double r = ((double) rand() / (RAND_MAX));
                    if (r<reactivation_rate){
                        multiArea[a].Completed = false;
                        contained[a] = 0;
                    }
                }
            }
            arena_update_counter=500;
        }
        else{
            arena_update_counter--;
        }
    }


/* Speak to the other ALF */
        /* --------- SERVER --------- */
        if (MODE=="SERVER"){
            /* Send posotion of each robot and the chosen direction */
            /* Transformation for expressing coordinates in 4 characters: origin translated to bottom right corner to have only positive values, then get first 2 digit after the comma */
            std::string pos = std::to_string(cKilobotPosition.GetX()+0.5);
            std::string pos2 = pos.substr(2,2);
            outputBuffer.append(pos2);
            pos = std::to_string(cKilobotPosition.GetY()+0.5);
            pos2 = pos.substr(2,2);
            outputBuffer.append(pos2);
            /* append 0 for no preferred direction, 1 for left, 2 for right */
            outputBuffer.append(std::to_string(unKilobotID)); //SOSTITUIRE CON IL BIT DESIDERATO
        }

    if (unKilobotID == 0){
        /* --------- CLIENT --------- */
        if (MODE=="CLIENT"){
            /* Build the message for the other ALF */
            if (initializing==false){
                outputBuffer = "A";
                for (int k=0; k<num_of_areas; k++){
                    if(multiArea[k].Completed==true){
                        outputBuffer.append("1");
                    }
                    else{
                        outputBuffer.append("0");
                    }
                }
            }
            else{
                initializing=false;
            }
        }

        /* Send the message to the other ALF*/
        if (MODE == "SERVER"){
            send(clientSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
        if (MODE == "CLIENT"){
            send(serverSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
        outputBuffer = "";  
    }


/* Task check*/
    if (initializing==false){
        if(MODE=="CLIENT"){
            for (int i=0;i<num_of_areas;i++){ 
                Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
                if((fDistance < (multiArea[i].Radius*1)) && (multiArea[i].Completed == false)){
                    multiArea[i].Completed=true;
                }
            }
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
        tKilobotMessage.m_sType = (int)m_vecKilobotStates_transmit[unKilobotID];     //state
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


REGISTER_LOOP_FUNCTIONS(CALFClientServer, "kilobot_ALF_cre_loop_function")