#ifndef CAMERA_REPORT_MONITOR_HPP_
#define CAMERA_REPORT_MONITOR_HPP_

#include "sfr.hpp"
#include <vector>

class CameraReportMonitor : public TimedControlTask<void>
{
public:
    CameraReportMonitor(unsigned int offset);
    File imgFile;
    void execute();
    void create_camera_report(int fragment_number, uint8_t serial_number);
    void add_possible_command();
};
#endif
