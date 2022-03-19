#include "ButtonMonitor.hpp"

ButtonMonitor::ButtonMonitor(unsigned int offset) : TimedControlTask<void>(offset) {}

void ButtonMonitor::execute()
{
    sfr::button::pressed = digitalRead(constants::button::button_pin);

    if (!sfr::button::pressed && sfr::camera::powered == true && sfr::camera::take_photo == false && sfr::camera::deployed == false) {
        delay(110);
        sfr::camera::take_photo = true;
        Serial.println("Button released, taking photo!");
        sfr::camera::deployed = true;
    }

#ifdef VERBOSE
    Serial.print("Button: ");
    Serial.print(sfr::button::pressed);
    Serial.println("(1 yes, 0 no)");
#endif
}