#include "kilobot_ALF_dhtf.h"


namespace{
const int port = 7001;

// environment setup
const double arena_size = 1.0;
const double kKiloDiameter = 0.033;
const double distance_threshold = arena_size/2.0 - 2.0*kKiloDiameter;
const int max_area_id = 15;

// wall avoidance stuff
const CVector2 up_direction (0.0, -1.0);
const CVector2 down_direction (0.0, 1.0);
const CVector2 left_direction (1.0, 0.0);
const CVector2 right_direction (-1.0, 0.0);
const int proximity_bits = 8;

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
        GetNodeAttribute(tModeNode,"reactivation_timer",kRespawnTimer);
        
        /* Select areas */
        srand (random_seed);
        
        /* GENERATE RANDOM IDs AND RANDOM HARD TASK for server and client*/
        std::default_random_engine re;
        re.seed(random_seed);
        std::vector<int> activated_areas;
        std::vector<int> hard_tasks_vec;
        std::vector<int> hard_tasks_client_vec;
        otherColor.resize(desired_num_of_areas);

        vCompletedTime.resize(desired_num_of_areas);
        std::fill(vCompletedTime.begin(),vCompletedTime.end(), 0.0);
        

        num_of_areas = desired_num_of_areas;
        /* Active IDs */
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

        /* Hard task for the server */
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
        
        

        /* Hard task for the client */
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
        
        std::cout<< "***********Active areas*****************\n";
        for(int ac_ar : activated_areas){
            std::cout<<ac_ar<<'\t';
        }
        std::cout<<std::endl;

        std::cout<< "Hard task server id\n";
        for(int h_t : hard_tasks_vec){
            std::cout<<h_t<<'\t';
        }
        std::cout<<std::endl;

        std::cout<< "Hard task client id\n";
        for(int h_t_c : hard_tasks_client_vec){
            std::cout<<h_t_c<<'\t';
        }
        std::cout<<std::endl;
        
        /* 0-1 vector indicatind if the active area is hard or soft type */
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


        for(int s_task : server_task_type)
        {
            initialise_buffer.append(std::to_string(s_task));
        }
        for(int c_task : client_task_type)
        {
            initialise_buffer.append(std::to_string(c_task));
        }

        // std::cout << "initialise_buffer: " << initialise_buffer << std::endl; 

        //Remove the extra multiArea loaded from .argos file
        for(int i=0; i<multiArea.size(); i++)
        {
            // fill server own colors
            if( std::find(activated_areas.begin(), activated_areas.end(), multiArea[i].id) == activated_areas.end() )
            {
                multiArea.erase(multiArea.begin()+i);
                i-= 1;
            }
            else
            {
                if ( std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), multiArea[i].id) != hard_tasks_vec.end() )
                {
                    multiArea[i].Color=argos::CColor::RED;
                }
                else
                {
                    multiArea[i].Color=argos::CColor::BLUE;
                } 
                    
                //fill client othercolor
                if ( std::find(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end(), multiArea[i].id) != hard_tasks_client_vec.end() )
                {
                    otherColor[i] = 1;
                }       
            }   

        }

        // Print active areas id and colour
        // std::cout<<"Area id \t colour\n";
        // for(int i=0; i<multiArea.size(); i++)
        // {
        //     std::cout<<multiArea[i].id<<'\t'<<multiArea[i].Color<<'\n';
        // }

        // Print other colour
        // std::cout<<"Client id \t colour\n";
        // for(int i=0; i<multiArea.size(); i++)
        // {
        //     std::cout<<multiArea[i].id<<'\t'<<otherColor[i]<<'\n';
        // }
    }

    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 30);     //set to 0 the 30 elements in storeBuffer
    initialised = false;

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
    otherColor = std::vector<int>(num_of_areas,0);

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
    else if (MODE == "CLIENT"){
        bytesReceived = recv(serverSocket, inputBuffer, 30, MSG_DONTWAIT);
    }

    if ((bytesReceived != -1) || (bytesReceived != 0))
    {
        /* Save the received string in a vector, for having data available until next message comes */
        for (int i=0; i<bytesReceived; i++){
            storeBuffer[i] = inputBuffer[i];
        }
        // Print received message
        // std::cout<<storeBuffer<<std::endl;
        std::string my_string (inputBuffer);
        if(!my_string.empty())
            std::cout << "Received:" << my_string << std::endl;
    }
    else {
        //std::cout << "not receiving" << std::endl;
    }

    // Print received message
    // std::cerr<<"Recv_str "<<storeBuffer<<std::endl;
    /* --------- CLIENT --------- */
    if (MODE=="CLIENT"){
        /* Initialize the tasks selected by the server */
        if ((storeBuffer[0]==73)&&(initialised==false)){    //73 is the ASCII binary for "I"
            /*choice of areas*/
            

            std::string storebuffer(storeBuffer);
            storebuffer.erase(storebuffer.begin());
            num_of_areas = storebuffer.size()/3;
            otherColor.resize(num_of_areas);
            std::string server_task(storebuffer.begin() + num_of_areas, storebuffer.begin() + 2*num_of_areas);
            std::string client_task(storebuffer.begin() + 2*num_of_areas, storebuffer.end());
            std::cout << server_task << std::endl;
            std::cout << client_task << std::endl;
            std::cout<< "num of areas: "<< num_of_areas << std::endl;

            std::vector<int> active_areas;
            for(int i=0; i<num_of_areas;i++)
            {
                active_areas.push_back(storebuffer[i]-97);
                // std::cout << "storebuffer[i] "<< storebuffer[i] << std::endl;
            }

            std::cout<< "Active areas: \n";        
            for(int id : active_areas)
            {
                std::cout << id << std::endl;
            }
            
            for(int i=0; i<multiArea.size(); i++)
            {
                if( std::find(active_areas.begin(), active_areas.end(), multiArea[i].id) == active_areas.end())
                {
                    multiArea.erase(multiArea.begin()+i);
                    i-= 1;
                }

                /*fill othercolor field*/
                if(server_task[i] == '1')
                    otherColor[i] = 1;

                /*fill own color field */
                if(client_task[i] == '1')
                    multiArea[i].Color = argos::CColor::RED;
            }
            
            std::cout<< "Multi areas: \n";        
            for(int id : active_areas)
            {
                std::cout << id << std::endl;
            }

            // std::cout<<"Recv_str "<<storeBuffer<<std::endl;
            initialised=true;
        }

        /* Align to server arena */
        if ((storeBuffer[0]==65)&&(initialised==true)){ //65 is the ASCII binary for "A"
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
        /* Reactivation areas check */
        for (int i=0; i<num_of_areas; i++){
            if((multiArea[i].Completed == true) &&
               (m_fTimeInSeconds - vCompletedTime[i] >= kRespawnTimer))
            {
                multiArea[i].Completed = false;
                contained[i] = 0;
            }
        }
        /* Task completeness check */
        if (storeBuffer[0]==84){ //84 is the ASCII binary for "T"
            for (int j=0; j<num_of_areas; j++){
                if ((storeBuffer[j+1]-48 == 1) && multiArea[j].Completed == false){
                    if (otherColor[j]==kRED){
                        if ((multiArea[j].Color==argos::CColor::RED)&&(contained[j]>=6)){
                            multiArea[j].Completed = true;
                            vCompletedTime[j] = m_fTimeInSeconds;
                            std::cout<<"red-red task completed"<<std::endl;
                        }
                        if ((multiArea[j].Color==argos::CColor::BLUE) && (contained[j] >= 2)) {
                            multiArea[j].Completed = true;
                            vCompletedTime[j] = m_fTimeInSeconds;
                            std::cout<<"blue-red task completed"<<std::endl;
                        }                        
                    }
                    if (otherColor[j]==kBLUE){
                        if ((multiArea[j].Color==argos::CColor::RED)&&(contained[j]>=6)){
                            multiArea[j].Completed = true;
                            vCompletedTime[j] = m_fTimeInSeconds;
                            std::cout<<"red-blue task completed"<<std::endl;
                        }
                        if ((multiArea[j].Color==argos::CColor::BLUE) && (contained[j] >= 2)) {
                            multiArea[j].Completed = true;
                            vCompletedTime[j] = m_fTimeInSeconds;
                            std::cout<<"blue-blue task completed"<<std::endl;
                        }                        
                    }                    
                }
            }
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
            if (initialised==true){
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
            else if (storeBuffer[0] == 82)
            {
                std::cout<<"ACK init by client*********\n";
                initialised = true;
            }
        }

        std::cout<<"Sending: ";
        /* Send the message to the other ALF*/
        if (MODE == "SERVER"){
            if(initialised == false){
                std::cout<<initialise_buffer<<std::endl;
                send(clientSocket, initialise_buffer.c_str(), initialise_buffer.size() + 1, 0);
            }
            else{
                // std::cout<<"mando update\n";
                std::cout<<outputBuffer<<std::endl;
                send(clientSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
            }
        }

        if (MODE == "CLIENT"){
            std::string client_str;

            if(initialised == false){
                client_str = "Missing parameters";
                std::cout<<client_str<<std::endl;
                send(serverSocket, client_str.c_str(), client_str.size() + 1, 0);
            }
            else if(storeBuffer[0]== 73)        //73 is the ASCII binary for "I"
            {
                client_str = "Received parameters";
                std::cout<<client_str<<std::endl;                
                send(serverSocket, client_str.c_str(), client_str.size() + 1, 0);
            }
            else
            {
                std::cout<<outputBuffer<<std::endl;
                send(serverSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
            }
            
        }   
    }


/* State transition*/
    if (initialised==true){
        switch (m_vecKilobotStates_ALF[unKilobotID]) {
            case OUTSIDE_AREAS : {
                /* Check if the kilobot is entered in a task area */
                for (int i=0;i<num_of_areas;i++){ 
                    Real fDistance = Distance(cKilobotPosition, multiArea[i].Center);
                    if((fDistance < (multiArea[i].Radius*1)) && (multiArea[i].Completed == false)){     //*1 is a threshold, to include the boarder increase it
                        m_vecKilobotStates_ALF[unKilobotID] = INSIDE_AREA;
                        /* Check LED color to understand if the robot is leaving or it is waiting for the task */
                        if (GetKilobotLedColor(c_kilobot_entity) != CColor::BLUE){
                            // std::cout<< "inside area = "<< multiArea[i].id << std::endl;
                            /* Check the area color to understand the requirements of the task */
                            if (multiArea[i].Color==argos::CColor::RED){
                                if(augmented_knowledge==true){
                                    if (otherColor[i]==kRED){
                                        request[unKilobotID] = kRR;
                                        // std::cout<<"red-red task 50s\n";
                                    }
                                    if (otherColor[i]==kBLUE){
                                        request[unKilobotID] = kRB;
                                        // std::cout<<"red-blue task 30s\n";
                                    }
                                }
                                else{
                                    request[unKilobotID] = kRB;
                                    // std::cout<<"unknown\n";
                                }
                            }
                            if (multiArea[i].Color==argos::CColor::BLUE){
                                if(augmented_knowledge==true){
                                    if (otherColor[i]==kRED){
                                        request[unKilobotID] = kBR;
                                        // std::cout<<"blue-red task 20s\n";
                                    }
                                    if (otherColor[i]==kBLUE){
                                        request[unKilobotID] = kBB;
                                        // std::cout<<"blue-blue task 10s\n";
                                    }
                                }
                                else
                                {
                                    request[unKilobotID] = kBB;
                                    // std::cout<<"unknown\n";
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
                if (GetKilobotLedColor(c_kilobot_entity) == CColor::BLUE){
                    m_vecKilobotStates_ALF[unKilobotID] = LEAVING;
                    contained[whereis[unKilobotID]] -= 1;
                }
                /* Else check if the task has been completed */
                if (multiArea[whereis[unKilobotID]].Completed == true){
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
                    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
                    whereis[unKilobotID] = -1;
                }
            break;
            }
        }

        // switch (m_vecKilobotStates_ALF[unKilobotID]) {
        //     case OUTSIDE_AREAS : {
        //         std::cout<<"Outside\n";
        //     break;
        //     }
        //     case INSIDE_AREA : {
        //         std::cout<<"Inside\n";
        //     break;
        //     }
        //     case LEAVING : {
        //         std::cout<<"Leaving\n";
        //     break;
        //     }
        //     default:
        //         std::cout<<"Error no state";
        //     break;
        // } 
    }
}

CVector2 CALFClientServer::VectorRotation2D (Real angle, CVector2 vec){
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector (kx, ky);
    return rotated_vector;
}

std::vector<int> CALFClientServer::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors/2.0);
    std::vector<int> proximity_values;

    for(int i=0; i<num_sectors; i++)
    {
        CVector2 sector_dir_a = VectorRotation2D( (kOrientation+M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_b = VectorRotation2D( (kOrientation+M_PI_2 - (i+1) * sector), left_direction);

        if( obstacle_direction.DotProduct(sector_dir_a) >= 0.0 || obstacle_direction.DotProduct(sector_dir_b) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
}

void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    // std::cerr<< "**Active areas**\n";
    // for (auto item : otherColor)
    // {
    //     if(item == 0)
    //         std::cerr<<'b'<<'\t';
    //     else if (item == 1)
    //     {
    //         std::cerr<<'r'<<'\t';
    //     }
    //     else
    //     {
    //         std::cerr<<"Error, not known color\n";
    //     }
        
    // }
    // std::cerr<<"\n";

    //Print active areas
    // if(MODE == "SERVER"){
    //     std::cout<< "***********Active areas*****************\n";
    //     for(auto ac_ar : multiArea){
    //         std::cout<<ac_ar.id<<'\t';
    //     }
    //     std::cout<<std::endl;
    // }

    /********* WALL AVOIDANCE STUFF *************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    CVector2 cKilobotPosition = GetKilobotPosition(c_kilobot_entity);
    CRadians cKilobotOrientation = GetKilobotOrientation(c_kilobot_entity);
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);

    // std::cerr<<unKilobotID<<'\t'<<cKilobotPosition<<std::endl;
    if( fabs(cKilobotPosition.GetX()) > distance_threshold ||
        fabs(cKilobotPosition.GetY()) > distance_threshold )
    {
        std::vector<int> proximity_vec;

        if(cKilobotPosition.GetX() > distance_threshold)
        {
            // std::cerr<<"RIGHT\n";
            proximity_vec = Proximity_sensor(right_direction, cKilobotOrientation.GetValue(), proximity_bits);
        }
        else if(cKilobotPosition.GetX() < -1.0*distance_threshold)
        {
            // std::cerr<<"LEFT\n";
            proximity_vec = Proximity_sensor(left_direction, cKilobotOrientation.GetValue(), proximity_bits);
        }
        
        if(cKilobotPosition.GetY() > distance_threshold)
        {
            // std::cerr<<"UP\n";
            if(proximity_vec.empty())
                proximity_vec = Proximity_sensor(up_direction, cKilobotOrientation.GetValue(), proximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, cKilobotOrientation.GetValue(), proximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform( proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());
                
                proximity_vec = elementwiseOr;
            }
            
        }
        else if(cKilobotPosition.GetY() < -1.0*distance_threshold)
        {
            // std::cerr<<"DOWN\n";
            if(proximity_vec.empty())
                proximity_vec = Proximity_sensor(down_direction, cKilobotOrientation.GetValue(), proximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, cKilobotOrientation.GetValue(), proximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform( proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());
                
                proximity_vec = elementwiseOr;
            }
        }

        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x,int y) {return (x << 1) + y;} );
        // To turn off the wall avoidance decomment this
        proximity_sensor_dec = 0;
        
        /** Print proximity values */
        // std::cerr<<"kID:"<< unKilobotID <<" sensor ";
        // for(int item : proximity_vec)
        // {
        //     std::cerr<< item <<'\t';
        // }
        // std::cerr<<std::endl;

        // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;
    }
    
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    bool bMessageToSend = false;
    
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg){
        return;
    }
    else{
        /* Compose the message for a kilobot */
        tKilobotMessage.m_sID = unKilobotID;                                            //ID of the receiver
        tKilobotMessage.m_sType = (int)m_vecKilobotStates_ALF[unKilobotID];             //state
        tKilobotMessage.m_sData = 0;                                 

        //entry msg when random walking        
        if ( (GetKilobotLedColor(c_kilobot_entity) != CColor::BLUE) && 
             (GetKilobotLedColor(c_kilobot_entity) != CColor::RED) &&
             ((int)m_vecKilobotStates_ALF[unKilobotID] == INSIDE_AREA))
        {
            bMessageToSend = true;
            tKilobotMessage.m_sData = request[unKilobotID];         //requirement (timer) for the area where it is

            // std::cerr<<"inside for "<< request[unKilobotID] << "\n";
        }

        //exit msg when inside
        if ( (GetKilobotLedColor(c_kilobot_entity) == CColor::RED) && 
             ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS))
        {
            bMessageToSend = true;
            // std::cerr<<"sending outside from inside\n";
        }

        if ((GetKilobotLedColor(c_kilobot_entity) == CColor::BLUE) && ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS)){   //exit msg when task completed
            bMessageToSend = true;
            // std::cerr<<"sending outside from leaving\n";
        }

        if( (fabs(cKilobotPosition.GetX()) > distance_threshold || fabs(cKilobotPosition.GetY()) > distance_threshold) && 
            ((int)m_vecKilobotStates_ALF[unKilobotID] != INSIDE_AREA) )
        {
            tKilobotMessage.m_sData = proximity_sensor_dec;
            bMessageToSend = true;
            // std::cerr<<"sending COLLIDING\n";
        }
        // bMessageToSend=true;      //use this line to send msgs always   
    }

    if (bMessageToSend){
        
        /** Print id and type of message sent to the kilobot */
        // switch (tKilobotMessage.m_sType) {
        //     case OUTSIDE_AREAS : {
        //         std::cout<<"Outside ";
        //     break;
        //     }
        //     case INSIDE_AREA : {
        //         std::cout<<"Inside ";
        //     break;
        //     }
        //     case LEAVING : {
        //         std::cout<<"Leaving ";
        //     break;
        //     }
        //     default:
        //         std::cout<<"Error no state ";
        //     break;
        // } 
        // std::cout<<"to kID:"<<unKilobotID<<std::endl;

        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

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
    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */
    
    for(int i=0; i<multiArea.size(); i++)
    {
        if(multiArea[i].Completed == false){
            Real fDistance = Distance(vec_position_on_plane,multiArea[i].Center);
            if(fDistance<multiArea[i].Radius){
                cColor=multiArea[i].Color;
            }   
        }
    }
    

    return cColor;
}


REGISTER_LOOP_FUNCTIONS(CALFClientServer, "kilobot_ALF_dhtf_loop_function")
