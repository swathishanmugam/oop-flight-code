#ifndef ACS_MONITOR_HPP_
#define ACS_MONITOR_HPP_

#include "sfr.hpp"

class ACSMonitor : public TimedControlTask<void>
{
public:
    ACSMonitor(unsigned int offset);
    void execute();
    void IMUOffset(float temp, float voltage, float pwmX, float pwmY, float pwmZ);
    StarshotACS0ModelClass rtObj;
};

#endif