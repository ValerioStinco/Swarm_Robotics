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
    GetNodeAttribute(tModeNode,"ip_addr",IP_ADDR);
    GetNodeAttribute(tModeNode,"random_seed",random_seed);
    GetNodeAttribute(tModeNode,"desired_num_of_areas",desired_num_of_areas);
    GetNodeAttribute(tModeNode,"communication_range",communication_range);
    GetNodeAttribute(tModeNode,"reactivation_rate",reactivation_rate);

    lenMultiArea=num_of_areas;

    if (MODE=="SERVER"){
        GetNodeAttribute(tModeNode,"vision_range",vision_range);
    }
    /* Randomly select the desired number of tasks between the available ones, set color and communicate them to the server */
    if (MODE=="CLIENT"){
        srand (random_seed);
        while(num_of_areas>desired_num_of_areas){
            for (int b=0;b<lenMultiArea;b++){
                if(multiArea[b].Completed==false){
                    double r = ((double) rand() / (RAND_MAX));
                    if (r<0.5){
                        multiArea[b].Completed=true;
                        num_of_areas--;
                        break;
                    }
                }
            }
        }
    }

    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 2000);
    arena_update_counter = 500;
    flag=0;
    outputBuffer = "";

    /* Opening communication port */
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    int port = 54000;
    std::string ipAddress = IP_ADDR;
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);
    
    if(MODE=="SERVER"){
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
    actual_orientation = std::vector<int>(num_of_kbs,0);
    command = std::vector<int>(num_of_kbs,0);
    visible_blue = std::vector<int>(num_of_kbs,0);
    visible_red = std::vector<int>(num_of_kbs,0);
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
        if (multiArea[ai].Center.GetX()<0){
            multiArea[ai].Color = argos::CColor::BLUE;
        }
        else{
            multiArea[ai].Color = argos::CColor::RED;
        }
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
    CRadians cKilobotOrientation = GetKilobotOrientation(c_kilobot_entity);

/* Listen for the other ALF communication */
    memset(inputBuffer, 0, 2000);
    if (MODE == "SERVER"){
        bytesReceived = recv(clientSocket, inputBuffer, 2000, MSG_DONTWAIT);
    }
    if (MODE == "CLIENT"){
        bytesReceived = recv(serverSocket, inputBuffer, 2000, MSG_DONTWAIT);
    }
    if ((bytesReceived == -1) || (bytesReceived == 0)){
        //std::cout << "not receiving" << std::endl;
    }
    else 
    {
        /* Save the received string in a vector, for having data available until next message comes */
        for (int i=0; i<2000; i++){
            storeBuffer[i] = inputBuffer[i];
        }
        //std::cout<<storeBuffer<<std::endl;
    }

    /* --------- SERVER --------- */
    if (MODE=="SERVER"){
        /* Align to server arena */
        for (int a=0; a<num_of_areas; a++){
            if (storeBuffer[a]-48 == 0) {
                multiArea[a].Completed = false;
            }
            else{
                multiArea[a].Completed = true;
            }
        }
    }

    /* --------- CLIENT --------- */
    if (MODE=="CLIENT"){
//************************************************************************************
//************************************************************************************
        multiTransmittingKilobot.resize(20);
        int j=0;
        //std::cout<<(int)(cKilobotOrientation.GetValue()*10)<<std::endl;

        command[unKilobotID]=0;
        for (int i=0;i<20;i++){          
            //acquire data of flying robots
            multiTransmittingKilobot[i].xCoord=(10*(storeBuffer[j]-48))+storeBuffer[j+1]-48;
            multiTransmittingKilobot[i].yCoord=(10*(storeBuffer[j+2]-48))+storeBuffer[j+3]-48;
            multiTransmittingKilobot[i].commit=storeBuffer[j+4]-48;
            //check command from flying robots
            //if zero skip the robot, else chek position
            if (multiTransmittingKilobot[i].commit==0){
                continue;
            }
            else{
                //check if a ground robot is under the cone of transmission of a flying robot ((MAX 20 FLYING AT THE MOMENT)
                float xdisp=multiTransmittingKilobot[i].xCoord-(25*(cKilobotPosition.GetX()+1));    //25* perchè moltiplico per 100 per considerare solo 2 decimali, poi divido per 4 per allineare le arene (una quadrupla dell'altra)
                float ydisp=multiTransmittingKilobot[i].yCoord-(25*(cKilobotPosition.GetY()+1));
                float displacement=sqrt((xdisp*xdisp)+(ydisp*ydisp));
                if (displacement<communication_range){
                    //check if the flying robot is in the semiplane opposit to its commitment
                    if(multiTransmittingKilobot[i].xCoord<=25){
                        if(multiTransmittingKilobot[i].commit==1){
                            command[unKilobotID]=1;     //robot at south, committed for north
                        }
                    }
                    else{
                        if(multiTransmittingKilobot[i].commit==2){
                            command[unKilobotID]=2;     //robot at north, committed for south
                        }
                    }
                }
            }
            j=j+5;
        }
    }
//************************************************************************************
//************************************************************************************

/* Speak to the other ALF */
    if (unKilobotID == 0){
        /* --------- CLIENT --------- */
        if (MODE=="CLIENT"){
            /* Build the message for the other ALF */
            outputBuffer = "";
            for (int k=0; k<lenMultiArea; k++){
                if(multiArea[k].Completed==true){
                    outputBuffer.append("1");
                }
                else{
                    outputBuffer.append("0");
                }
            }
            //std::cout<<"outbuffer "<<outputBuffer<<std::endl;
        }

        /* Send the message to the other ALF*/
        if (MODE == "SERVER"){
            send(clientSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
        if (MODE == "CLIENT"){
            send(serverSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
        //std::cout<<"pos and commit:\t"<< outputBuffer << std::endl;
        outputBuffer = "";
    }
        /* --------- SERVER --------- */
    if (MODE=="SERVER"){
        /* Send position of each robot and the chosen direction */
        /* Transformation for expressing coordinates in 4 characters: origin translated to bottom right corner to have only positive values, then get first 2 digit after the comma */
        std::string pos = std::to_string(cKilobotPosition.GetX()+0.25);
        std::string pos2 = pos.substr(2,2);
        outputBuffer.append(pos2);
        pos = std::to_string(cKilobotPosition.GetY()+0.25);
        pos2 = pos.substr(2,2);
        outputBuffer.append(pos2);
        /* append 0 for no preferred direction, 1 for left, 2 for right */
        if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::RED){
            outputBuffer.append(std::to_string(1)); //kilobot committed for north
        }
        if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::BLUE){
            outputBuffer.append(std::to_string(2)); //kilobot committed for south
        }
    }



/* Task check*/
    if(MODE=="CLIENT"){
        for (int i=0;i<lenMultiArea;i++){ 
            Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
            if((fDistance < (multiArea[i].Radius*1)) && (multiArea[i].Completed == false)){
                multiArea[i].Completed=true;
                /* Reactivate tasks to keep their number constant */
                for (int a=0; a<lenMultiArea; a++){
                    if (multiArea[a].Completed == true){
                        double r = ((double) rand() / (RAND_MAX));
                        if (r<reactivation_rate){
                            multiArea[a].Completed = false;
                            break;
                        }
                    }
                }
            }
        }
    }
    if(MODE=="SERVER"){
        visible_blue[unKilobotID]=0;
        visible_red[unKilobotID]=0;
        for (int i=0;i<lenMultiArea;i++){
            Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
            if((fDistance < 0.1) && (multiArea[i].Completed == false)){
                if (multiArea[i].Color == argos::CColor::RED){
                    visible_red[unKilobotID]++;
                }
                else if (multiArea[i].Color == argos::CColor::BLUE){
                    visible_blue[unKilobotID]++;
                }
            }          
        }
    }
}


void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    CVector2 cKilobotPosition = GetKilobotPosition(c_kilobot_entity);
    decisionMessage KilobotDecisionMsg, EmptyDecisionMsg, DecisionMsg;
    bool bMessageToSend = false;
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);

////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (MODE=="SERVER"){
        //CI SONO DA DEFINIRE I NUOVI CAMPI DEL MESSAGGIO




        if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg){
            return;
        }
        else{
            float r_on=0, r_off=0, b_on=0, b_off=0;
            for (int i=0;i<lenMultiArea;i++){ 
                Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
                if(fDistance <= vision_range){
                    if (multiArea[i].Color==argos::CColor::RED){
                        if(multiArea[i].Completed==true){
                            r_off++;
                        }
                        else if(multiArea[i].Completed==false){
                            r_on++;
                        }
                    }
                    else if (multiArea[i].Color==argos::CColor::BLUE){
                        if(multiArea[i].Completed==true){
                            b_off++;
                        }
                        else if(multiArea[i].Completed==false){
                            b_on++;
                        }
                    }
                }
            }
            std::cout<<"lenMultiArea: "<<lenMultiArea<<std::endl;
            std::cout<<r_on<<" - "<<r_off<<" ------ "<<b_on<<" - "<<b_off<<std::endl;
            KilobotDecisionMsg.ID = unKilobotID;
            KilobotDecisionMsg.resource_red = 100*(r_on/(r_on+r_off));
            KilobotDecisionMsg.resource_blue = 100*(b_on/(b_on+b_off));
            std::cout<<KilobotDecisionMsg.resource_red<<" - "<<KilobotDecisionMsg.resource_blue<<std::endl;
            m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
            bMessageToSend = true;
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (MODE=="CLIENT"){
        CRadians cKilobotOrientation = GetKilobotOrientation(c_kilobot_entity);

        /*determine kilobot orientation*/
        actual_orientation[unKilobotID]=(int)(cKilobotOrientation.GetValue()*10);
        if(actual_orientation[unKilobotID]<0){
            actual_orientation[unKilobotID]=(-1*actual_orientation[unKilobotID])+100;
        }
        //std::cout<<"act orientation: "<<actual_orientation[unKilobotID]<<std::endl;
        std::cout<<"command: "<<command[unKilobotID]<<std::endl;

        if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg){
            return;
        }
        else{
            /* Compose the message for a kilobot */
            tKilobotMessage.m_sID = unKilobotID;                            //ID of the receiver
            tKilobotMessage.m_sType = (int)command[unKilobotID];            //state
            tKilobotMessage.m_sData = actual_orientation[unKilobotID];      //orientation of the robot
            if ((command[unKilobotID]!=0)&&(GetKilobotLedColor(c_kilobot_entity)== argos::CColor::BLACK)){
                std::cout<<"sending command"<<std::endl;
                bMessageToSend = true;
            }
            m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
            
        }
    }
    if (bMessageToSend){
        for (int i=0; i<9; ++i) {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;

        EmptyDecisionMsg.ID = 255;
        EmptyDecisionMsg.resource_red = 0;
        EmptyDecisionMsg.resource_blue = 0;
        for (int i=0; i<3; ++i) {
            /* Packing the message */
            if (MODE=="CLIENT"){
                if (i == 0){
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
            else if (MODE=="SERVER"){
                if (i == 0){
                    DecisionMsg = KilobotDecisionMsg;
                } else{
                    DecisionMsg = EmptyDecisionMsg;
                }
                m_tMessages[unKilobotID].data[i*3] = DecisionMsg.ID;
                m_tMessages[unKilobotID].data[1+i*3] = DecisionMsg.resource_red;
                m_tMessages[unKilobotID].data[2+i*3] = DecisionMsg.resource_blue;
            }
        }
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }
}


CColor CALFClientServer::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */
    for (int i=0; i<lenMultiArea; i++){
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