#include "ACSMonitor.hpp"

ACSMonitor::ACSMonitor(unsigned int offset)
    : TimedControlTask<void>(offset)
{
    using namespace sfr::acs; 
    starshotObj.initialize(kane_damper_c, kane_Id,
    ampfactor, max_current, csarea,
    num_loops, wdx, wdy, wdz);
}

void ACSMonitor::execute()

{
    //loading adjusted values into the ACS model
    starshotObj.rtU.w[0] = sfr::imu::gyro_x * 3.14159 / 180.0;
    starshotObj.rtU.w[1] = sfr::imu::gyro_y * 3.14159 / 180.0;
    starshotObj.rtU.w[2] = sfr::imu::gyro_z * 3.14159 / 180.0;
    IMUOffset(sfr::temperature::temp_c, sfr::battery::voltage, sfr::acs::pwmX, sfr::acs::pwmY, sfr::acs::pwmZ);
    starshotObj.rtU.magneticfield[0] = sfr::imu::mag_x;
    starshotObj.rtU.magneticfield[1] = sfr::imu::mag_y;
    starshotObj.rtU.magneticfield[2] = sfr::imu::mag_z;
    starshotObj.step();

    /*if(sfr::fault::check_acc_x && sfr::fault::check_acc_y && sfr::acs::mode != acs_mode_type::off){
        if(sfr::imu::gyro_x < 0 && sfr::imu::gyro_y < 0){
            sfr::acs::mode = acs_mode_type::point;
        }
        else{
            sfr::acs::mode = acs_mode_type::detumble;
        }
    }
    */

    switch(sfr::acs::mode){
        case acs_mode_type::detumble:
            sfr::acs::currentX = starshotObj.rtY.detumble[0];
            sfr::acs::currentY = starshotObj.rtY.detumble[1];
            sfr::acs::currentZ = starshotObj.rtY.detumble[2];
            break;
        case acs_mode_type::point:
            sfr::acs::currentX = starshotObj.rtY.point[0];
            sfr::acs::currentY = starshotObj.rtY.point[1];
            sfr::acs::currentZ = starshotObj.rtY.point[2];
            break;
        case acs_mode_type::off:
            sfr::acs::currentX = 0;
            sfr::acs::currentY = 0;
            sfr::acs::currentZ = 0;
            break;
    }

    DataLog();
}

void ACSMonitor::IMUOffset(float temp, float voltage, float pwmX, float pwmY, float pwmZ)
{
    using namespace constants::acs; 

    /*Offset Contributions from PWM (ex: pwmX_oX is contribution of X mag to offset x)*/
    float pwmX_ox = pwmX_ox_1*pwmX + pwmX_ox_2*pow(pwmX,2) + pwmX_ox_3*pow(pwmX,3); 
    float pwmX_oy = pwmX_oy_1*pwmX + pwmX_oy_2*pow(pwmX,2) + pwmX_oy_3*pow(pwmX,3); 
    float pwmX_oz = pwmX_oz_1*pwmX + pwmX_oz_2*pow(pwmX,2) + pwmX_oz_3*pow(pwmX,3); 
    
    float pwmY_ox = pwmY_ox_1*pwmY + pwmY_ox_2*pow(pwmY,2) + pwmY_ox_3*pow(pwmY,3); 
    float pwmY_oy = pwmY_oy_1*pwmY + pwmY_oy_2*pow(pwmY,2) + pwmY_oy_3*pow(pwmY,3); 
    float pwmY_oz = pwmY_oz_1*pwmY + pwmY_oz_2*pow(pwmY,2) + pwmY_oz_3*pow(pwmY,3); 

    float pwmZ_ox = pwmZ_ox_1*pwmZ + pwmZ_ox_2*pow(pwmZ,2) + pwmZ_ox_3*pow(pwmZ,3); 
    float pwmZ_oy = pwmZ_oy_1*pwmZ + pwmZ_oy_2*pow(pwmZ,2) + pwmZ_oy_3*pow(pwmZ,3); 
    float pwmZ_oz = pwmZ_oz_1*pwmZ + pwmZ_oz_2*pow(pwmZ,2) + pwmZ_oz_3*pow(pwmZ,3); 
    /*******************************************/
    /*Voltage Adjustment Coefficients (ex: volX_ox = coef for pwmX_oX)*/
    float volX_ox = volX_ox_1 * voltage + volX_ox_c; 
    float volX_oy = volX_oy_1 * voltage + volX_oy_c; 
    float volX_oz = volX_oz_1 * voltage + volX_oz_c ; 
 
    float volY_ox = volY_ox_1 * voltage + volY_ox_c; 
    float volY_oy = volY_oy_1 * voltage + volY_oy_c; 
    float volY_oz = volY_oz_1 * voltage + volY_oz_c; 

    float volZ_ox = volZ_ox_1 * voltage + volZ_ox_c; 
    float volZ_oy = volZ_oy_1 * voltage + volZ_oy_c; 
    float volZ_oz = volZ_oz_1 * voltage + volZ_oz_c; 
    /*******************************************/
    /*Temperature Offset Terms*/
    float temp_x = temp_x_1*temp + temp_x_2*pow(temp,2) + temp_x_3*pow(temp, 3) + temp_x_c; 
    float temp_y = temp_y_1*temp + temp_y_2*pow(temp,2) + temp_y_3*pow(temp, 3) + temp_y_c; 
    float temp_z = temp_z_1*temp + temp_z_2*pow(temp,2) + temp_z_3*pow(temp, 3) + temp_z_c; 
    /*******************************************/
     /*Total Offsets*/
    float xoffset = volX_ox * pwmX_ox+ volY_ox * pwmY_ox + volZ_ox * pwmZ_ox + temp_x + hardiron_x; 
    float yoffset = volX_oy * pwmX_oy + volY_oy * pwmY_oy + volZ_oy * pwmZ_oy + temp_y + hardiron_y; 
    float zoffset = volX_oz * pwmX_oz + volY_oz * pwmY_oz + volZ_oz * pwmZ_oz + temp_z + hardiron_z; 
    /*******************************************/
    /* Finally, adjust magnetometer readings*/
    sfr::imu::mag_x -= xoffset; 
    sfr::imu::mag_x -= yoffset; 
    sfr::imu::mag_x -= zoffset; 

}
