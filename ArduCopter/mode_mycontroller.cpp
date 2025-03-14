#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h>
// #include "mycontroller_usercode.h"
#include <AP_GPS/AP_GPS.h>


bool ModeMyController::init(bool){
    // hal.serial(2)->begin(115200);

    return true;
}

void ModeMyController::run(){
    // hal.serial(2)->printf("%1d,%6.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%6.2f,%6.2f,%7.2f,%7.2f,%7.2f,%4d,%4d,%4d,%4d_",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw,H_yaw_rate,H_throttle,PWM1,PWM2,PWM3,PWM4);
    hal.console->printf("Hello World from Mycontroller mode \n");

}
