#include "MissionManager.hpp"
#include "sfr.hpp"
#include <unity.h>

void reset(MissionManager mission_manager)
{
    sfr::mission::current_mode = sfr::mission::boot;
    mission_manager.execute();
    TEST_ASSERT_EQUAL(sfr::mission::boot->id(), sfr::mission::current_mode->id());
}

void test_valid_initialization()
{
    MissionManager mission_manager(0);
    mission_manager.execute();
    // TEST_ASSERT_EQUAL(sfr::mission::boot->id(), sfr::mission::current_mode->id());
}

void test_exit_boot()
{
    MissionManager mission_manager(0);
    reset(mission_manager);

    // after max boot time, transition to aliveSignal
    sfr::mission::max_boot_time = 500;
    delay(sfr::mission::max_boot_time);
    mission_manager.execute();
    TEST_ASSERT_EQUAL(sfr::mission::aliveSignal->id(), sfr::mission::current_mode->id());
}

void test_exit_alive_signal()
{
    MissionManager mission_manager(0);
    reset(mission_manager);

    // exit if voltage sensor fails
    sfr::battery::voltage_average->set_invalid();
    mission_manager.execute();
    TEST_ASSERT_EQUAL(sfr::mission::lowPowerAliveSignal->id(), sfr::mission::current_mode->id());
}

int test_mission_manager()
{
    UNITY_BEGIN();
    RUN_TEST(test_valid_initialization);
    // RUN_TEST(test_exit_boot);
    // RUN_TEST(test_exit_alive_signal);
    return UNITY_END();
}

#ifdef DESKTOP
int main()
{
    return test_mission_manager();
}
#else
#include <Arduino.h>
void setup()
{
    delay(2000);
    Serial.begin(9600);
    test_mission_manager();
}

void loop() {}
#endif