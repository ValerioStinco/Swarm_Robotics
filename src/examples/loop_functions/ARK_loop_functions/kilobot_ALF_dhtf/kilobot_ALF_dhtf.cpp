#include "kilobot_ALF_dhtf.h"


namespace{
const int port = 7001;
const double kKiloDiameter = 0.033;
const int max_area_id = 15;
}

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
    GetNodeAttribute(tModeNode,"timeout_const",TIMEOUT_CONST);
    GetNodeAttribute(tModeNode,"augmented_knowledge",augmented_knowledge);


    /* Randomly select the desired number of tasks between the available ones, set color and communicate them to the client */
    if (MODE=="SERVER"){
        GetNodeAttribute(tModeNode,"random_seed",random_seed);
        GetNodeAttribute(tModeNode,"desired_num_of_areas",desired_num_of_areas);
        GetNodeAttribute(tModeNode,"hard_tasks",hard_tasks);
        GetNodeAttribute(tModeNode,"reactivation_rate",reactivation_rate);
        outputBuffer="I";
        /* Select areas */
        srand (random_seed);
        
        std::default_random_engine re;
        re.seed(random_seed);
        std::vector<int> activated_areas;
        std::vector<int> hard_tasks_vec;
        std::vector<int> hard_tasks_client_vec;

        while (activated_areas.size() < desired_num_of_areas)
        {
            if(desired_num_of_areas-1 > max_area_id)
            {
                std::cerr<<"Requested more areas then the available ones, WARNING!";
            }
        
            std::uniform_int_distribution<int> distr(0, max_area_id);
            int random_number;
            do{
                random_number = distr(re);
            }while (std::find(activated_areas.begin(), activated_areas.end(), random_number) != activated_areas.end());
            activated_areas.push_back(random_number);
        }
        std::sort(activated_areas.begin(), activated_areas.end());

        while (hard_tasks_vec.size() < hard_tasks)
        {
            std::uniform_int_distribution<int> distr(0, max_area_id);int random_number;
            do{
                random_number = distr(re);
            }while (std::find(activated_areas.begin(), activated_areas.end(), random_number) == activated_areas.end() ||
                    std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), random_number) != hard_tasks_vec.end());
            hard_tasks_vec.push_back(random_number);
        }
        std::sort(hard_tasks_vec.begin(), hard_tasks_vec.end());
        

        while (hard_tasks_client_vec.size() < hard_tasks)
        {
            std::uniform_int_distribution<int> distr(0, max_area_id);int random_number;
            do{
                random_number = distr(re);
            }while (std::find(activated_areas.begin(), activated_areas.end(), random_number) == activated_areas.end() ||
                    std::find(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end(), random_number) != hard_tasks_client_vec.end());
            hard_tasks_client_vec.push_back(random_number);
        }
        std::sort(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end());
        
        for(int ac_ar : activated_areas){
            std::cout<<ac_ar<<'\t';
        }
        std::cout<<std::endl;
        for(int h_t : hard_tasks_vec){
            std::cout<<h_t<<'\t';
        }
        std::cout<<std::endl;
        for(int h_t_c : hard_tasks_client_vec){
            std::cout<<h_t_c<<'\t';
        }
        std::cout<<std::endl;
        
        // preparint initialise ("I") server message
        std::vector<int> server_task_type (activated_areas.size(), 0);
        std::vector<int> client_task_type (activated_areas.size(), 0);

        initialise_buffer = "I";

        for(int i=0; i<activated_areas.size(); i++)
        {  
            int char_id = 97+activated_areas[i];    // 97 is a in ASCII table
            char A = static_cast<char>(char_id);
            std::string s(1, A);
            initialise_buffer.append(s);

            if(std::find(hard_tasks_vec.begin(),hard_tasks_vec.end(), activated_areas[i]) != hard_tasks_vec.end())
                server_task_type[i] = 1;

            if(std::find(hard_tasks_client_vec.begin(),hard_tasks_client_vec.end(), activated_areas[i]) != hard_tasks_client_vec.end())
                client_task_type[i] = 1;

        }

        // std::cout << "server " << server_task_type;
        // std::cout << "client " << client_task_type;

        for(uint s_task : server_task_type)
        {
            initialise_buffer.append(std::to_string(s_task));
        }
        for(uint c_task : client_task_type)
        {
            initialise_buffer.append(std::to_string(c_task));
        }

        std::cout << "initialise_buffer: " << initialise_buffer << std::endl; 

     
    }

    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 30);     //set to 0 the 30 elements in storeBuffer
    arena_update_counter = 500;     //timer, when espired there is reactivation probability
    initializing = true;

    /* Socket initialization, opening communication port */
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    std::string ipAddress = IP_ADDR;
    sockaddr_in hint;
    hint.sin_family = AF_INET;
	hint.sin_addr.s_addr = INADDR_ANY;
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
        if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0){
            std::cout << host << " connected on port " << service << std::endl;
            std::cout << "Somebody has connected on port " << service << std::endl;
        }
        else{
            inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
            std::cout << host << " connected on port " << ntohs(client.sin_port) << std::endl;
            std::cout << "Somebody has connected on port " << service << std::endl;
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
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    /* Initialization of kilobots variables */
    request = std::vector<UInt8>(m_tKilobotEntities.size(),0);
    whereis = std::vector<SInt8>(m_tKilobotEntities.size(),-1);
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
        multiArea[ai].id = ai;
        multiArea[ai].Completed = false;
        multiArea[ai].Color = argos::CColor::BLUE;
        otherColor[ai] = 1;
    }

    /* Initialization of areas variables */
    contained = std::vector<UInt8>(num_of_areas, 0);
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
        //std::cout<<storeBuffer<<std::endl;
    }

    /* --------- CLIENT --------- */
    if (MODE=="CLIENT"){
        /* Initialize the tasks selected by the server */
        if ((storeBuffer[0]==73)&&(initializing==true)){    //73 is the ASCII binary for "I"
            /*choice of areas*/
            for (int a=1; a<30; a++)
            {
                int n = storeBuffer[a]-97;
                if (n>=0){
                    for (int b = n; b < num_of_areas; b++){
                        multiArea[b] = multiArea[b + 1];
                    }
                    num_of_areas--;
                }
            }

            std::string storebuffer(storeBuffer);
            std::cout << "storebuffer "<< storeBuffer << std::endl;
            std::cout<< "storeBuffer" << std::endl;
            for(int i=0; i<30; i++)
            {
                std::cout<< storeBuffer[i];
            }
            std::cout<<std::endl;

            std::cout << "storeBuffer "<< storeBuffer << std::endl;
            std::cout << "storebuffer "<< storebuffer << std::endl;
            storebuffer.erase(storebuffer.begin());
            std::cout << "storebuffer "<< storebuffer << std::endl;
            std::vector<int> areas_id;
            for(int j=0; j<storebuffer.size()/3; j++){
                areas_id.push_back(storebuffer[j]-97);
            }
            std::cout<<"Selected areas : ";
            for(int id : areas_id){
                std::cout<< id << '\t';
            }
            std::cout<<std::endl;

            std::vector<SVirtualArea> multiArea_temp;

            //for(int i=1; i=num_of_areas;i++)
            
            
            
            
            
            
            
            /*fill othercolor field*/
            for (int c=num_of_areas+1; c<(2*num_of_areas)+1; c++){
                if (storeBuffer[c]==49){
                    otherColor[c-num_of_areas-1]=2;
                }
                else if (storeBuffer[c]==48){
                    otherColor[c-num_of_areas-1]=1;
                }
            }
            /*fill color field*/
            for (int c=(2*num_of_areas)+1;c<(3*num_of_areas)+1;c++){
                if (storeBuffer[c]==49){
                    multiArea[c-(2*num_of_areas)-1].Color=argos::CColor::RED;
                }
                else if (storeBuffer[c]==48){
                    multiArea[c-(2*num_of_areas)-1].Color=argos::CColor::BLUE;
                }
            }
            // std::cout<<"Recv_str "<<storeBuffer<<std::endl;
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


    /* --------- SERVER --------- */
    if (MODE=="SERVER"){
        /* Task completeness check */
        if (storeBuffer[0]==84){ //84 is the ASCII binary for "T"
            for (int j=0; j<num_of_areas; j++){
                if ((storeBuffer[j+1]-48 == 1)){
                    if (otherColor[j]==2){
                        if ((multiArea[j].Color==argos::CColor::RED)&&(contained[j]>=6)){
                            multiArea[j].Completed = true;
                            std::cout<<"red-red task completed"<<std::endl;
                        }
                        if ((multiArea[j].Color==argos::CColor::BLUE) && (contained[j] >= 2)) {
                            multiArea[j].Completed = true;
                            std::cout<<"blue-red task completed"<<std::endl;
                        }                        
                    }
                    if (otherColor[j]==1){
                        if ((multiArea[j].Color==argos::CColor::RED)&&(contained[j]>=6)){
                            multiArea[j].Completed = true;
                            std::cout<<"red-blue task completed"<<std::endl;
                        }
                        if ((multiArea[j].Color==argos::CColor::BLUE) && (contained[j] >= 2)) {
                            multiArea[j].Completed = true;
                            std::cout<<"blue-blue task completed"<<std::endl;
                        }                        
                    }                    
                }
            }
        }
        /* Reactivate tasks already comlpeted (server routine) */
        if (arena_update_counter == 0){
            srand (random_seed);
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
    if (unKilobotID == 0){          // just to speak to the other ARK once for each cycle
        /* --------- CLIENT --------- */
        if (MODE=="CLIENT"){
            /* Build the message for the other ALF */
            outputBuffer = "T"; //"T" indicates that the message is related to task completeness
            for (int k=0; k<num_of_areas; k++){
                /* Write 1 or 2 if the requirements of the area are satisfied for the sender, else write 0 */
                if (multiArea[k].Color==argos::CColor::RED){
                    if (contained[k] >= 6) {
                        outputBuffer.append("1");   //hard task completed
                    }
                    else {
                        outputBuffer.append("0");
                    }
                }
                else if (multiArea[k].Color==argos::CColor::BLUE){
                    if (contained[k] >= 2) {
                        outputBuffer.append("1");   //easy task completed
                    }
                    else {
                        outputBuffer.append("0");
                    }            
                }
            }
            //std::cout<<outputBuffer<<std::endl;
        }

        /* --------- SERVER --------- */
        if (MODE=="SERVER"){
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
    }


/* State transition*/
    if (initializing==false){
        switch (m_vecKilobotStates_ALF[unKilobotID]) {
            case OUTSIDE_AREAS : {
                /* Check if the kilobot is entered in a task area */
                for (int i=0;i<num_of_areas;i++){ 
                    Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
                    if((fDistance < (multiArea[i].Radius*1)) && (multiArea[i].Completed == false)){     //*1 is a threshold, to include the boarder increase it
                        m_vecKilobotStates_transmit[unKilobotID] = INSIDE_AREA;
                        m_vecKilobotStates_ALF[unKilobotID] = INSIDE_AREA;
                        /* Check LED color to understand if the robot is leaving or it is waiting for the task */
                        if (GetKilobotLedColor(c_kilobot_entity) != argos::CColor::RED){
                            /* Check the area color to understand the requirements of the task */
                            if (multiArea[i].Color==argos::CColor::RED){
                                if(augmented_knowledge==true){
                                    if (otherColor[i]==2){
                                        request[unKilobotID] = 5*TIMEOUT_CONST;
                                    }
                                    if (otherColor[i]==1){
                                        request[unKilobotID] = 3*TIMEOUT_CONST;
                                    }
                                }
                                else{
                                    request[unKilobotID] = 3*TIMEOUT_CONST;
                                }
                            }
                            if (multiArea[i].Color==argos::CColor::BLUE){
                                if(augmented_knowledge==true){
                                    if (otherColor[i]==2){
                                        request[unKilobotID] = 2*TIMEOUT_CONST;
                                    }
                                    if (otherColor[i]==1){
                                        request[unKilobotID] = 1*TIMEOUT_CONST;
                                    }
                                }
                                else
                                {
                                    request[unKilobotID] = 1*TIMEOUT_CONST;
                                }
                            }
                            whereis[unKilobotID] = i;
                            contained[i] += 1;
                        }
                    }
                }
            break;
            }
            case INSIDE_AREA : {
                /* Check if the kilobot timer for colaboratos is expired */
                if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::RED){
                    m_vecKilobotStates_transmit[unKilobotID] = INSIDE_AREA;
                    m_vecKilobotStates_ALF[unKilobotID] = LEAVING;
                    contained[whereis[unKilobotID]] -= 1;
                }
                /* Else check if the task has been completed */
                else if (multiArea[whereis[unKilobotID]].Completed == true){
                    m_vecKilobotStates_transmit[unKilobotID] = OUTSIDE_AREAS;
                    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
                    contained[whereis[unKilobotID]] = 0;
                    whereis[unKilobotID] = -1;
                }
            break;
            }
            case LEAVING : {
                /* Case in which the robot is inside an area but it is moving */
                Real fDistance = Distance(cKilobotPosition, multiArea[whereis[unKilobotID]].Center);
                /* Check when the robot is back outside  */
                if (fDistance > (multiArea[whereis[unKilobotID]].Radius)){
                    m_vecKilobotStates_transmit[unKilobotID] = OUTSIDE_AREAS;
                    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
                    whereis[unKilobotID] = -1;
                }
            break;
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
        tKilobotMessage.m_sID = unKilobotID;                                            //ID of the receiver
        tKilobotMessage.m_sType = (int)m_vecKilobotStates_transmit[unKilobotID];        //state
        tKilobotMessage.m_sData = request[unKilobotID];                                 //requirement (timer) for the area where it is
        if ((GetKilobotLedColor(c_kilobot_entity) != argos::CColor::BLUE) && (GetKilobotLedColor(c_kilobot_entity) != argos::CColor::RED) && ((int)m_vecKilobotStates_transmit[unKilobotID] == INSIDE_AREA)){          //entry msg when random walking
            bMessageToSend = true;
        }
        else if ((GetKilobotLedColor(c_kilobot_entity) == argos::CColor::RED) && ((int)m_vecKilobotStates_transmit[unKilobotID] == OUTSIDE_AREAS)){     //exit msg when leaving
            bMessageToSend = true;
        }
        else if ((GetKilobotLedColor(c_kilobot_entity) == argos::CColor::BLUE) && ((int)m_vecKilobotStates_transmit[unKilobotID] == OUTSIDE_AREAS)){   //exit msg when task completed
            bMessageToSend = true;
        }
        //bMessageToSend=true;      //use this line to send msgs always
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
            //std::cout<<" robot "<<tMessage.m_sID<<" "<<tMessage.m_sType<<std::endl;
        }
        //std::cout<<"payload: "<<tKilobotMessage.m_sData<<std::endl;
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
