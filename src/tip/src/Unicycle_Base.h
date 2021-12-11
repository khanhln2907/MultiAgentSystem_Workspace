#pragma once
#include "commonType.h"
#include <stdint.h>
#include "ros/ros.h"

class Unicycle_Base
{
public:
    Unicycle_Base();
    ~Unicycle_Base();

    // Initialize Agent with positions, input thresholds, ...
    virtual void begin(UnicycleState InitState, UnicycleInput inputMax);
    
    virtual void updateState(UnicycleState newState);
    // Update Agent's control input through these methods
    void updateInput(UnicycleInput newInput);

    // Child Class must define this method so that each agent has an unique ID
    // virtual uint16_t get_AgentID() = 0;

protected:

    // Used this method to control the maximal available control input
    bool cutoff(UnicycleInput& ControlInput);

    UnicycleState _currentState;
    UnicycleInput _inputThreshold;

private:
    
};

