#include "ACSMonitor.hpp"
#include "math.h"
ACSMonitor::ACSMonitor(unsigned int offset)
    : TimedControlTask<void>(offset)
{
    rtObj.initialize();
}

void ACSMonitor::execute()
{
    rtObj.rtU.angularvelocity[0] = sfr::imu::gyro_x * 3.14159 / 180.0;
    rtObj.rtU.angularvelocity[1] = sfr::imu::gyro_y * 3.14159 / 180.0;
    rtObj.rtU.angularvelocity[2] = sfr::imu::gyro_z * 3.14159 / 180.0;
    IMUOffset(sfr::temperature::temp_c, sfr::battery::voltage, sfr::acs::pwm1, sfr::acs::pwm2, sfr::acs::pwm3);
    rtObj.rtU.Bfield_body[0] = sfr::imu::mag_x;
    rtObj.rtU.Bfield_body[1] = sfr::imu::mag_y;
    rtObj.rtU.Bfield_body[2] = sfr::imu::mag_z;
    rtObj.step();

    /*if(sfr::fault::check_acc_x && sfr::fault::check_acc_y && sfr::acs::mode != acs_mode_type::off){
        if(sfr::imu::gyro_x < 0 && sfr::imu::gyro_y < 0){
            sfr::acs::mode = acs_mode_type::point;
        }
        else{
            sfr::acs::mode = acs_mode_type::detumble;
        }
    }

    switch(sfr::acs::mode){
        case acs_mode_type::detumble:
            sfr::acs::current1 = rtObj.rtY.detuble[0];
            sfr::acs::current2 = rtObj.rtY.detuble[1];
            sfr::acs::current3 = rtObj.rtY.detuble[2];
            break;
        case acs_mode_type::point:
            sfr::acs::current1 = rtObj.rtY.point[0];
            sfr::acs::current2 = rtObj.rtY.point[1];
            sfr::acs::current3 = rtObj.rtY.point[2];
            break;
        case acs_mode_type::off:
            sfr::acs::current1 = 0;
            sfr::acs::current2 = 0;
            sfr::acs::current3 = 3;
            break;
    }*/
}

void ACSMonitor::IMUOffset(float temp, float voltage, float pwm1, float pwm2, float pwm3)
{
    /*
    1,2,3 --> x,y,z 
    xoffset = temp_x offset * voltage_x offset * (PWM_x offset + PWM_y offset + PWM_z offset)
    yoffset = temp_y offset * voltage_y offset * (PWM_x offset + PWM_y offset + PWM_z offset)
    zoffset = temp_z offset * voltage_z offset * (PWM_x offset + PWM_y offset + PWM_z offset)

    new mag_x += xoffset
    new mag_y += xoffset
    new mag_z += xoffset

    */

    /*Offset Contributions from PWM (ex: pwm1_X is contribution of x mag to X offset)*/
    float pwm1_X = 0.063563305433041769609836535437353*pwm1 + 0.0000024534801053495480847952553427049*pow(pwm1,2) 
    - 0.00000004048183650236316959656411693276*pow(pwm1,3); 
    float pwm1_Y = 0.0000080605967200896503065685530509121*pow(pwm1,2) - 0.016797609228196081626593993973984*pwm1 
    + 0.000000012114204069568563619616825333992*pow(pwm1,3); 
    float pwm1_Z = 0.0017958858966789052746815258387869*pwm1 + 0.000029717338935004530305191913774898*pow(pwm1,2) 
    + 0.000000011774600680023870762861215393501*pow(pwm1,3); 
    
    float pwm2_X = - 0.029758606652877828135927984476439*pwm2 + 0.0000036655843236024399974936228280287*pow(pwm2,2)  
    + 0.000000023498369011861784752980125408166*pow(pwm2,3); 
    float pwm2_Y = 0.042815004635511522135260520371958*pwm2 + 0.000015182122409661965729810693959134*pow(pwm2,2) 
    - 0.000000015456536073234678752023188506685*pow(pwm2,3); 
    float pwm2_Z = 0.000023898623196982134755635487910475*pow(pwm2,2) - 0.064033443011555668533674179343507*pwm2 
    + 0.000000058017983244733767109801910464087*pow(pwm2,3); 

    float pwm3_X = 0.049277911306007247949079186355448*pwm3 + 0.0000001823555642185477635673483036774*pow(pwm3,2) 
    - 0.000000025703762479892763114309778196137*pow(pwm3,3); 
    float pwm3_Y = 0.015385071524851799337763047503813*pwm3 + 0.000018702657754292684140007621551582*pow(pwm3,2) 
    - 0.0000000031989424202132932197155153338734*pow(pwm3,3); 
    float pwm3_Z = 0.000014901521761221710052694455761468*pow(pwm3,2) - 0.018823855991873226362054438709492*pwm3 
    + 0.000000014948952011168461910939879054225*pow(pwm3,3); 


    /*Voltage Adjustment Coefficients (ex: vol_1X = coef for pwm1_X)*/
    float vol_1X = 0.12375707365261180867327588256536 * voltage + 0.48022029065903040357224129322547; 
    float vol_1Y = -(0.54592452576565059280663991683131* voltage - 3.2928830082157324897878876506915); 
    float vol_1Z = (0.75132926778397910193914377979025*voltage - 4.155582924692712228144403875119); 

    float vol_2X = -(0.27909194272927493419007019171813 * voltage - 2.1721861594629547235982948052162); 
    float vol_2Y = 0.16676521878709328946499592169343* voltage + 0.29958608109420818424701712888759; 
    float vol_2Z = - (0.14064119928172391235146520553687*voltage + 0.40930696301675956812384613674515); 

    float vol_3X = 0.15986532071754780144314062996388 * voltage + 0.32856565298629923393880935415171; 
    float vol_3Y = 0.38192997034365841187064582326419* voltage - 0.60410587544336532985671245770959; 
    float vol_3Z = -(0.54213732000179207555231037777489*voltage - 1.2769767440075267173197035866545); 
    /*******************************************/
    
    /*Temperature Offset Terms*/
    float temp_x = - 0.45645709660067923518766974666505*temp + 0.0052611925857588160565514456834535*pow(temp,2) 
    + 0.000010529099180900968306677685515371*pow(temp, 3) + 7.3908341297177386763905815314502; 
    float temp_y = 0.32652431483093125041961002352764*temp - 0.0049475302634079910621411890758736*pow(temp,2) 
    - 0.0000015243327341598422207456613175491*pow(temp, 3) - 4.5352297389919185022222336556297; 
    float temp_z = 0.091402346089728289668663308020768*temp - 0.00061841171449482097276922898387852*pow(temp,2) 
    + 0.0000069097524971878865738986012778877*pow(temp, 3) - 1.7573124306246634684924856628641; 
    /*******************************************/

    /*Total Soft Iron Offsets*/
    float xoffset = vol_1X * pwm1_X+ vol_2X * pwm2_X + vol_3X * pwm3_X + temp_x
    float yoffset = vol_1Y * pwm1_Y + vol_2Y * pwm2_Y + vol_3Y * pwm3_Y + temp_y
    float yoffset = vol_1Z * pwm1_Z + vol_2Z * pwm2_Z + vol_3Z * pwm3_Z + temp_z
    /*******************************************/
    /*Add in Hard-Iron Offsets*/
    xoffset += -13.65
    yoffset += 36.058
    zoffset += 6.928
    /*******************************************/
    /* Finally, adjust magnetometer readings*/
    mag_x -= xoffset
    mag_y -= yoffset
    mag_z -= zoffset

}
