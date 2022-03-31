#include "PhotoresistorMonitor.hpp"

PhotoresistorMonitor::PhotoresistorMonitor(unsigned int offset) : TimedControlTask<void>(offset) {}

void PhotoresistorMonitor::execute()
{
    int val = analogRead(constants::photoresistor::pin);
    sfr::button::pressed = digitalRead(constants::button::button_pin);

#ifdef VERBOSE
    Serial.print("Photoresistor: ");
    Serial.print(val);
    Serial.println(" (0-1023 scale)");
#endif
    if (val > constants::photoresistor::light_val) {
        sfr::photoresistor::covered = false;
        Serial.print("photoresistor activation time: ");
        Serial.println(millis());
    } else {
        sfr::photoresistor::covered = true;
    }

    if (!sfr::button::pressed) {
        Serial.println("Button trigger time: ");
        Serial.print(millis());
    }

    /*
        if (!sfr::photoresistor::covered && sfr::camera::powered == true && sfr::camera::take_photo == false && sfr::camera::deployed == false) {
            delay(110);
            sfr::camera::take_photo = true;
            Serial.println("Deployment detected, taking photo!");
            sfr::camera::deployed = true;
        }
    */
}