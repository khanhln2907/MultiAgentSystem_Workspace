#include "Unicycle_BLF_Agent.h"
#include <time.h>
#include <stdio.h>

#define printID ROS_INFO("", _myID);

Unicycle_BLF_Agent::Unicycle_BLF_Agent(){
    ROS_INFO("AgentNode constructed.");
};

Unicycle_BLF_Agent::~Unicycle_BLF_Agent(){
    ROS_INFO("AgentNode destructed.");
};

void Unicycle_BLF_Agent::begin(BLF_CoverageAgentConfig_t initConfig){
    // Assign the control parameter
    _controllParameter = initConfig.controlParameter;
    
     // Assign the initial pose and the maximum control input
    Unicycle_Base::begin(initConfig.initPose, _controllParameter.inputMax);

    // Init the vector to carry the information of neighbors
    _nTotalAgents = initConfig.nTotalAgents;
    _myID = initConfig.myID;
    for (int i = 0; i < _nTotalAgents - 1; i++){
        // initialize the list of neighbor agent with the init ID as -1 to indicate that they are not yet registered by any other agent
        BLF_CoverageMessage tmpInfoInstance;
        tmpInfoInstance.TransmitterID = -1;
        _neighborArr.push_back(tmpInfoInstance);
    }
    ROS_INFO("ID: %d --> Initialized succesfully. Total operating agents: %d", _myID, _nTotalAgents);
};

bool Unicycle_BLF_Agent::updateNeighbor(BLF_CoverageMessage newInfo){
     

    // Search for neighrbor fixID in the array
    for (int i = 0; i < _nTotalAgents; i++){
        // Update the information if the registered neighbor agent is found
        if(newInfo.TransmitterID == _neighborArr.at(i).TransmitterID ){
            _neighborArr.at(i) = newInfo;
            //ROS_INFO("ID: %d: Neighbor ID %d found", _myID, _neighborArr.at(i).TransmitterID);
            return true;
        // Assign new neighbor if the current ID at that slot is not yet assigned (the actual value is -1)
        } else if (_neighborArr.at(i).TransmitterID == -1) {
            _neighborArr.at(i) = newInfo;
            ROS_INFO("ID: %d: Neighbor ID %d registered !", _myID, _neighborArr.at(i).TransmitterID);
            //ROS_INFO("Update List: %d %d %d!",
            //    _neighborArr[0].TransmitterID, _neighborArr[1].TransmitterID, _neighborArr[2].TransmitterID);
            return true;
        }
    }
    // If we go until here, the total of operating agents exceed the maximum size of vector neighborArr;
	ROS_ERROR("Can not assign new neighbor Info. Check size of neighbor vector!"); 
    return false;
}

BLF_CoverageMessage Unicycle_BLF_Agent::getPublishedInfo(){
    BLF_CoverageMessage ret;
    ret.TransmitterID = _myID;
    ret.AgentPose = _currentState;
    ret.VirtualCenter = _curVirtualCenter;
    ret.V_BLF = _curV_BLF;
    return ret;
};

void Unicycle_BLF_Agent::updateState(UnicycleState newState){
    // Update the pose
    Unicycle_Base::updateState(newState);
    // Compute and update the virtual center based on the updated state
    static uint32_t gamma = _controllParameter.controlGain;
    static float v0 = _controllParameter.constTranslationAndOrbitVelocity.TranVel;
    static float w0 = _controllParameter.constTranslationAndOrbitVelocity.AngularVel;
    _curVirtualCenter.x = _currentState.x - v0 /w0 * sin(_currentState.theta); 
    _curVirtualCenter.y = _currentState.y + v0 /w0 * cos(_currentState.theta);
}

