#pragma once
#include "commonType.h"
#include <vector>


#include "Unicycle_BLF_Agent.h"

class CentralizedMonitor 
{
public:
    CentralizedMonitor();
    void begin(uint16_t nAgent);
    void publish();

private:
    std::vector<BLF_CoverageMessage> _agentArray;
    
    




};
