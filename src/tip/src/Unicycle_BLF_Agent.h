# pragma once
#include "Unicycle_Base.h"
#include "vector"

typedef struct BLF_CoverageMessage {
        int32_t TransmitterID;
        UnicycleState AgentPose;
        Pose2D VirtualCenter;
        float V_BLF;
        // And Something else
        // ...
        // Note that changing this protocol requires edit in the ROS Message
} BLF_CoverageMessage;


typedef struct BLF_ControlConfig_t{
        UnicycleInput inputMax;
        UnicycleInput constTranslationAndOrbitVelocity;         // Constant v_0 and w_0, which is configured at the beginning
        uint32_t controlGain;
}BLF_ControlConfig_t;


typedef struct BLF_CoverageAgentConfig_t {
        uint16_t nTotalAgents;
        int32_t myID;
        UnicycleState initPose;
        BLF_ControlConfig_t controlParameter;
} BLF_CoverageAgentConfig_t;

class Unicycle_BLF_Agent : public Unicycle_Base {
public:        
        Unicycle_BLF_Agent();
        
        ~Unicycle_BLF_Agent();

        // Initialize an agent with a const amount of max robots executing the mission
        // Provide each agent with a fix ID
        virtual void begin(BLF_CoverageAgentConfig_t initConfig);

        //void updateNeighborPosition(char neighborID, UnicycleState nbState);
        bool updateNeighbor(BLF_CoverageMessage newMsg);

        // Overloaded method to update the state and the position of its virtual center at the same time;
        virtual void updateState(UnicycleState newState);

        BLF_CoverageMessage getPublishedInfo();

        int32_t _myID;
private:
        // Variables for communication between Agents
        uint16_t _nTotalAgents;
        std::vector<BLF_CoverageMessage> _neighborArr;

        // Variables for computing the control input
        Pose2D _curVirtualCenter;       // Actual position of the virtual center, this is updated whenever agent obtain its position
        float _curV_BLF;                // Computed Barrier Lyapunov Result
        BLF_ControlConfig_t _controllParameter;
};