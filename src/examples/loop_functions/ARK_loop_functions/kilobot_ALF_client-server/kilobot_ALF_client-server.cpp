#include "kilobot_ALF_client-server.h"

int freedomTh = 8;

CALFClientServer::CALFClientServer() :
    m_unDataAcquisitionFrequency(10){
    bytesReceived = -1;
}


CALFClientServer::~CALFClientServer(){
}


void CALFClientServer::Init(TConfigurationNode& t_node) {
    CALF::Init(t_node);
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

    TConfigurationNode& tModeNode = GetNode(t_node, "functioning_mode");       //Read functioning mode: determine if CLIENT or SERVER
    GetNodeAttribute(tModeNode,"mode",MODE);

    /*variables initialization*/
    flag = std::vector<bool>(2,0);  //can be adjusted basing on how many flags are needed
    multiArea[0].Free=false;
    multiArea[1].Free=false;
    multiArea[2].Free=false;
    multiArea[3].Free=false;
    multiArea[4].Free=false;
    multiArea[5].Free=false;

    /*Opening communication port*/
    if(MODE=="SERVER"){
        int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        int port = 54000;
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
    }
    if(MODE=="CLIENT"){
        serverSocket = socket(AF_INET, SOCK_STREAM, 0); //socket for client-server communication
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
    if(MODE=="SERVER"){
        close(clientSocket);
    }
    if(MODE=="CLIENT"){
        close(serverSocket);
    }
}


void CALFClientServer::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecKilobotData.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    num_of_kbs=0;
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
        num_of_kbs = num_of_kbs+1;
    }
    inPlace = std::vector<bool>(num_of_kbs,0);
}


void CALFClientServer::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = RANDOM_WALKING;
    m_vecLastTimeMessaged[unKilobotID] = -1000;
}


void CALFClientServer::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    TConfigurationNodeIterator itAct;
    num_of_areas=0;
    for(itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct) {
        num_of_areas = num_of_areas+1;
    }
    multiArea.resize(num_of_areas);
    contained = std::vector<int>(num_of_areas,0);
    int i=0;
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


void CALFClientServer::GetExperimentVariables(TConfigurationNode& t_tree){
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}


void CALFClientServer::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
/*listen to the other ALF*/
    memset(buf, 0, 30);
    if(MODE=="SERVER"){
        bytesReceived = recv(clientSocket, buf, 30, MSG_DONTWAIT);
    }
    if(MODE=="CLIENT"){
        bytesReceived = recv(serverSocket, buf, 30, MSG_DONTWAIT);
    }

    if ((bytesReceived == -1)||(bytesReceived == 0)){
        //std::cout << "not receiving" << std::endl;
    }
    else 
    {
        for (int i=0; i<30; i++){
            bufStore[i]=buf[i];         //save buf in a permanent string every time a new message comes, to have data available until the next message
        }
        std::cout<<"String received: "<<bufStore<<std::endl;
        if (MODE=="CLIENT"){
            multiArea[bufStore[0]-48].Free = true;
        }
    }

/*Update kilobot state*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    if(unKilobotID==0){
        CColor LED = GetKilobotLedColor(c_kilobot_entity);
        std::cout<<"Colore: "<<LED<<std::endl;
        if(LED == argos::CColor::BLACK){
            std::cout<<"OK!"<<std::endl;
        }
        //std::cout<<"Tipo: "<<typeid(LED).name()<<std::endl;
    }

    if(MODE=="SERVER"){
        if (multiArea[target_index].Free == true){
            m_vecKilobotStates[unKilobotID]=RANDOM_WALKING;
        }
        if (m_vecKilobotStates[unKilobotID]==SEARCHING){    //When in SEARCHING MODE kilobots will stop as soon as they reach target area and the number of arrived unities is send via socket
            CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
            Real fDistance = Distance(cKilobotPosition, multiArea[target_index].Center);
            if((fDistance<(multiArea[target_index].Radius*1)) && (multiArea[target_index].Free == false)){
                m_vecKilobotStates[unKilobotID]=INSIDE_AREA;
                if (inPlace[unKilobotID]==0){
                    inPlace[unKilobotID]=1;
                    int placed = std::accumulate(inPlace.begin(), inPlace.end(), 0); //how many KBs are in position
                    if (placed>=freedomTh){
                        multiArea[target_index].Free=true;
                        std::string buf = std::to_string(target_index);
                        std::cout<<"Free area number "<<target_index<<std::endl;
                        send(clientSocket, buf.c_str(), buf.size() + 1, 0);
                    }
                }
            }
            if(multiArea[target_index].Free == true){
                m_vecKilobotStates[unKilobotID]=RANDOM_WALKING;
            }

        }
    }
    if(MODE=="CLIENT"){
        /*if (flag[0]==0){
            std::string buf1 = "333";       //turn on white LED: it means the client swarm started moving
            send(serverSocket, buf1.c_str(), buf1.size() + 1, 0);
            std::cout<<"Sending initial signal "<<buf1<<std::endl;
            flag[0] = 1;
        }*/
        for(int i=0;i<num_of_areas;i++){            
            CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
            Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
            if((fDistance<(multiArea[i].Radius*1)) && (multiArea[i].Free == false)){
                m_vecKilobotStates[unKilobotID]=INSIDE_AREA;
                m_vecKilobotData[unKilobotID].R=(multiArea[i].Color.GetRed())/85;       //get components of area color and divide by 85 to turn from 0-255 to 0-3 range
                m_vecKilobotData[unKilobotID].G=(multiArea[i].Color.GetGreen())/85;     //then assemble in a single number ALL to be transmitted
                m_vecKilobotData[unKilobotID].B=(multiArea[i].Color.GetBlue())/85;
                m_vecKilobotData[unKilobotID].ALL=(m_vecKilobotData[unKilobotID].B)+10*(m_vecKilobotData[unKilobotID].G)+100*(m_vecKilobotData[unKilobotID].R);
                if (inPlace[unKilobotID]==0){                               //say in which area each kilobot stops
                    contained[i-1]+=1;
                    inPlace[unKilobotID]=1;
                    int placed = std::accumulate(inPlace.begin(), inPlace.end(), 0); //how many KBs are in position

                    /*Sending data*/
                    if (placed==num_of_kbs && flag[1]==0){                                   //say how many kilobot there are in each area
                        std::string buf = "Distribution: ";
                        for(int k=0;k<num_of_areas;k++){
                            buf.append(" ");
                            buf.append(std::to_string(contained[k]));
                        }
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
                        std::cout<<"Most populated area: "<<max_index<<std::endl;
                        buf = maxs;                                            //use this to send the number of the target area
                        std::cout<<"Sending index "<<buf<<std::endl;
                        send(serverSocket, buf.c_str(), buf.size() + 1, 0);            //send the RGB color of the most populated area in client experiment
                        flag[1]=1;
                    }
                }
                break;
            }
            else{
                m_vecKilobotStates[unKilobotID]=RANDOM_WALKING;
            }
        }
    }
}


void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    bool bMessageToSend=false;
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return;
    }
    else{
        if(MODE=="SERVER"){
            if (bufStore[0]!=0){
                target_index = ((int)bufStore[0]-48);                   //ASCII conversion of char to int number
                std::string target_color = multiArea[target_index].RGBcolor;
                m_vecKilobotData[unKilobotID].ALL=atoi(target_color.c_str());    //use this when receiving area index
                if(m_vecKilobotStates[unKilobotID]==RANDOM_WALKING){
                    m_vecKilobotStates[unKilobotID]=SEARCHING;
                }
            }
            if (multiArea[target_index].Free == true){
                //m_vecKilobotStates[unKilobotID]==RANDOM_WALKING;
                m_vecKilobotData[unKilobotID].ALL=0;
            }
        }
        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID];
        tKilobotMessage.m_sData = (int)m_vecKilobotData[unKilobotID].ALL;
        bMessageToSend=true;
        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    }

    if(bMessageToSend){
        for (int i = 0; i < 9; ++i) {
            m_tMessages[unKilobotID].data[i]=0;
        }
        tEmptyMessage.m_sID=1023;
        tEmptyMessage.m_sType=0;
        tEmptyMessage.m_sData=0;
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
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    else{
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
    }
}


CColor CALFClientServer::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    for (int i=0; i<num_of_areas; i++){
        if (multiArea[i].Free == false){
            Real fDistance = Distance(vec_position_on_plane,multiArea[i].Center);
                if(fDistance<multiArea[i].Radius){
                    cColor=multiArea[i].Color;
                }
        }
    }
    return cColor;
}


REGISTER_LOOP_FUNCTIONS(CALFClientServer, "kilobot_ALF_client-server_loop_function")