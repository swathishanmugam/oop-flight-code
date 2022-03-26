#include "PhotoresistorMonitor.hpp"

PhotoresistorMonitor::PhotoresistorMonitor(unsigned int offset) : TimedControlTask<void>(offset) {}

void PhotoresistorMonitor::execute()
{
    int val = analogRead(constants::photoresistor::pin);
#ifdef VERBOSE_SENSOR
    Serial.print("Photoresistor: ");
    Serial.print(val);
    Serial.println(" (0-1023 scale)");
#endif
    if (val > constants::photoresistor::light_val) {
        sfr::photoresistor::covered = false;
    } else {
        sfr::photoresistor::covered = true;
    }
}