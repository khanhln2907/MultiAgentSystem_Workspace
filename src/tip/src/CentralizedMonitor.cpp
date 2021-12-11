#include "CentralizedMonitor.h"

CentralizedMonitor::CentralizedMonitor(){};

void CentralizedMonitor::begin(uint16_t nAgent){
    _agentArray.resize(nAgent);
};
