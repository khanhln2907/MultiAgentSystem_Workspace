#include "Unicycle_Base.h"
#include <algorithm>

Unicycle_Base::Unicycle_Base(){
;
};

Unicycle_Base::~Unicycle_Base(){
;
};

// Initialize Agent with positions, input thresholds, ...
void Unicycle_Base::begin(UnicycleState InitState, UnicycleInput inputMax){
    _currentState = InitState;
    _inputThreshold = inputMax;
};

void Unicycle_Base::updateState(UnicycleState newState){
    _currentState = newState;
};

// Update Agent's control input through these methods
void Unicycle_Base::updateInput(UnicycleInput newInput){
    cutoff(newInput);
    // Do something to publish the input ?
};

// Used this method to control the maximal available control input
bool Unicycle_Base::cutoff(UnicycleInput& ControlInput){
    bool isSaturated =      (abs(ControlInput.TranVel) > abs(_inputThreshold.TranVel) 
                            || abs(ControlInput.AngularVel) > abs(_inputThreshold.AngularVel));
    
    if (ControlInput.TranVel > _inputThreshold.TranVel){
        ControlInput.TranVel = _inputThreshold.TranVel;
    } else if (ControlInput.TranVel < -_inputThreshold.TranVel){
        ControlInput.TranVel = - _inputThreshold.TranVel;
    }

    if (ControlInput.AngularVel > _inputThreshold.AngularVel){
        ControlInput.AngularVel = _inputThreshold.AngularVel;
    } else if (ControlInput.AngularVel < -_inputThreshold.AngularVel){
        ControlInput.AngularVel = -_inputThreshold.AngularVel;
    }

    return isSaturated;
};