// Taken from Cornell's PAN

#include "Control Tasks/TimedControlTask.hpp"

sys_time_t TimedControlTaskBase::control_cycle_start_time;
unsigned int TimedControlTaskBase::control_cycle_count = 0;