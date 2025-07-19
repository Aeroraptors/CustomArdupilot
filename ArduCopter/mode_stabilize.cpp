#include "Copter.h"
#include <AP_Motors/AP_Motors.h>

#define PI 3.1415926535

////////    Define Variables to Get Pilot inputs

float pilot_roll        = 0.0;
float pilot_pitch       = 0.0;
float pilot_yaw         = 0.0;
float pilot_throttle    = 0.0;

float pilot_ch5         = 0.0;
float pilot_ch6         = 0.0;
float pilot_ch7         = 0.0;
float pilot_ch8         = 0.0;

float des_quad_roll     = 0.0;      // degrees
float des_quad_pitch    = 0.0;      // degrees
float des_quad_yaw      = 0.0;      // degrees
float des_quad_roll_dot = 0.0;      // degrees/second
float des_quad_pitch_dot= 0.0;      // degrees/second
float des_quad_yaw_dot  = 0.0;      // degrees/second

float des_quad_pitch_prev      = 0.0;      // degrees
float des_quad_roll_prev       = 0.0;      // degrees

float quad_roll         = 0.0;      // degrees 
float quad_pitch        = 0.0;      // degrees 
float quad_yaw          = 0.0;      // degrees 
float quad_roll_dot     = 0.0;      // degrees/second
float quad_pitch_dot    = 0.0;      // degrees/second
float quad_yaw_dot      = 0.0;      // degrees/second

float quad_x            = 0.0;      // m
float quad_y            = 0.0;      // m
float quad_z            = 0.0;      // m
float quad_x_dot        = 0.0;      // m/s
float quad_y_dot        = 0.0;      // m/s
float quad_z_dot        = 0.0;      // m/s

float des_quad_x        = 0.0;      // m
float des_quad_y        = 0.0;      // m
float des_quad_z        = 0.0;      // m
float des_quad_x_dot    = 0.0;      // m/s
float des_quad_y_dot    = 0.0;      // m/s
float des_quad_z_dot    = 0.0;      // m/s

float des_quad_x_dot_prev = 0.0;    // m/s
float des_quad_y_dot_prev = 0.0;    // m/s
float des_quad_z_dot_prev = 0.0;    // m/s

float quad_x_ini        = 0.0;      // m
float quad_y_ini        = 0.0;      // m
float quad_z_ini        = 0.0;      // m

Vector3f rpy(0.0, 0.0, 0.0);
Matrix3f R(1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0);
Vector3f Omega(0.0, 0.0, 0.0);

Vector3f rpyd(0.0, 0.0, 0.0);
Matrix3f Rd(1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
Vector3f Omegad(0.0, 0.0, 0.0);

////////    Define Variables to Initialize the PWM Signals

uint16_t PWM1_Custom = 1000;
uint16_t PWM2_Custom = 1000;
uint16_t PWM3_Custom = 1000;
uint16_t PWM4_Custom = 1000;

float mq    = 1.5;                            // mass of the drone
float J1    = 0.0113;                         // MOI about b1 axis
float J2    = 0.0133;                         // MOI about b2 axis
float J3    = 0.0187;                         // MOI about b3 axis
float g_acc = 9.81;                           // Gravitational acceleration

/////////////////////// Manual gain tuning  ///////////////////////

// Attitude Controller Gains
float KR1         = 0.3;  // You should tune this values
float KOmega1     = 13.5; // You should tune this values

float KR2         = 0.4;  // You should tunethis values
float KOmega2     = 19.0; // You should tune this values

float KR3         = 6.0;  // You should tune this values
float KOmega3     = 10.0; // You should tune this values

Matrix3f KR(
    KR1,0.0,0.0,
    0.0,KR2,0.0,
    0.0,0.0,KR3
);
Matrix3f KOmega(
    KOmega1,0.0,0.0,
    0.0,KOmega2,0.0,
    0.0,0.0,KOmega3
    );
// Intertia matrix
Matrix3f JJ(
    0.0113, 0.0 , 0.0,
    0.0, 0.0133,0.0,
    0.0,0.0,0.0187
    );

// Positional Controller Gains
float Kp_x = 4.0; // You should tune this values
float Kd_x = 2.0; // You should tune this values

float Kp_y = 4.0; // You should tune this values
float Kd_y = 2.0; // You should tune this values

float Kp_z = 9.0; // You should tune this values
float Kd_z = 5.0; // You should tune this values

Matrix3f Kxq(
    Kp_x, 0.0, 0.0,
    0.0, Kp_y, 0.0,
    0.0, 0.0, Kp_z);

Matrix3f Kxq_dot(
    Kd_x, 0.0, 0.0,
    0.0, Kd_y, 0.0,
    0.0, 0.0, Kd_z);

/* 
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{

    //////    Call function to read pilot inputs
    function_1_Get_Pilot_Input();
    function_3_Getting_Drone_States();

    if ( pilot_ch6 < 1500)
    {
        function_2_Bydefault_Stabilize_code();
        hal.console->printf("Hi From by Default Code\n");

        // Reset the states
        des_quad_x      = 0.0;
        des_quad_y      = 0.0;
        des_quad_z      = 0.0;
        des_quad_yaw    = 360.0 -  (ahrs.yaw_sensor)   / 100.0;     // degrees 
        des_quad_x_dot       = 0.0;
        des_quad_y_dot       = 0.0;
        des_quad_z_dot       = 0.0;
        des_quad_yaw_dot     = 0.0;

        quad_x_ini      =  inertial_nav.get_position_neu_cm().x / 100.0;         // m
        quad_y_ini      = -inertial_nav.get_position_neu_cm().y / 100.0;        // m
        quad_z_ini      =  inertial_nav.get_position_neu_cm().z / 100.0;         // m

    }
    else if (pilot_ch6 > 1500)
    {
        // hal.console->printf("Hi From Custom Created Code\n");
        
        function_5_Nonlinear_Geometric_Control();

        if (copter.motors->armed()){
            // PWM1_Custom = 1200;
            // PWM2_Custom = 1400;
            // PWM3_Custom = 1600;
            // PWM4_Custom = 1800;
            hal.console->printf("PWM1-%d,PWM2-%d,PWM3-%d,PWM4-%d\n",PWM1_Custom,PWM2_Custom,PWM3_Custom,PWM4_Custom);
    
        }else{
            PWM1_Custom = 1000;
            PWM2_Custom = 1000;
            PWM3_Custom = 1000;
            PWM4_Custom = 1000;
        }
    }
}

void ModeStabilize::function_1_Get_Pilot_Input()
{
    pilot_roll      = channel_roll->get_control_in();               // -4500 to 4500
    pilot_pitch     = channel_pitch->get_control_in();              // -4500 to 4500
    pilot_yaw       = channel_yaw->get_control_in();                // -4500 to 4500
    pilot_throttle  = channel_throttle->get_control_in() - 500;     //  -500 to  500

    // hal.console->printf("Roll- %3.3f, Pitch- %3.3f, Yaw- %3.3f, Throttle- %3.3f, ", pilot_roll, pilot_pitch, pilot_yaw, pilot_throttle);

    pilot_ch5       = RC_Channels::get_radio_in(CH_5);        // 1000 to 2000
    pilot_ch6       = RC_Channels::get_radio_in(CH_6);        // 1000 to 2000
    pilot_ch7       = RC_Channels::get_radio_in(CH_7);        // 1000 to 2000
    pilot_ch8       = RC_Channels::get_radio_in(CH_8);        // 1000 to 2000

    // hal.console->printf("Ch5- %3.3f, Ch6- %3.3f, Ch7- %3.3f, Ch8- %3.3f\n", pilot_ch5, pilot_ch6, pilot_ch7, pilot_ch8);

    // Get Desired Attitude of Drone

    des_quad_roll           = pilot_roll/100.0;
    des_quad_roll_dot       = (des_quad_roll - des_quad_roll_prev)/400.0;
    des_quad_roll_prev      = des_quad_roll;

    des_quad_pitch          = pilot_pitch/100.0;
    des_quad_pitch_dot      = (des_quad_pitch - des_quad_pitch_prev)/400.0;
    des_quad_pitch_prev     = des_quad_pitch;

    des_quad_yaw_dot        = -pilot_yaw / 100.0;
    float dt_yaw = 1.0/100.0;
    des_quad_yaw = wrap_360(des_quad_yaw + des_quad_yaw_dot * dt_yaw);

    //////// Getting Desired Position from roll and pitch angles

    //////// Common values
    float max_des_velocity_horizontal   = 5.0;      // (m/s)
    float max_des_velocity_vertical     = 5.0;      // (m/s)
    float slop_for_horizontal_scaling   = 45.0 / max_des_velocity_horizontal;
    float slop_for_vertical_scaling     = 500.0 / max_des_velocity_vertical;

    //////// Convert des_quad_pitch into desired velocity in x
        // Trim (-2 to 2) data. This will help preventing the drone to drift
            if (des_quad_pitch > -2.0 && des_quad_pitch < 2.0)
            {
                des_quad_pitch = 0.0;
            }
        // Start the values considering -2 and 2 as the origin
            if (des_quad_pitch < -2.0 && des_quad_pitch > -45.0)
            {
                des_quad_pitch = des_quad_pitch + 2.0;
            }
            if (des_quad_pitch > 2.0 && des_quad_pitch < 45.0)
            {
                des_quad_pitch = des_quad_pitch - 2.0;
            }

        // Scale the (-45 to 45) values to desired velocity limits
            des_quad_x_dot = des_quad_pitch / slop_for_horizontal_scaling;

        // First ordered low pass filter
            des_quad_x_dot      = 0.686 * des_quad_x_dot_prev + 0.314 * des_quad_x_dot;
            des_quad_x_dot_prev = des_quad_x_dot;
            des_quad_x          = des_quad_x + (des_quad_x_dot + des_quad_x_dot_prev) / 2 * 0.0025;

        // Put bounds on quadcopter's position
            des_quad_x          = function_6_Bounds_on_XY_des(des_quad_x);

    //////// Convert des_quad_roll into desired velocity in y
        // Trim (-2 to 2) data. This will help preventing the drone to drift
        if (des_quad_roll > -2.0 && des_quad_roll < 2.0)
        {
            des_quad_roll = 0.0;
        }
        // Start the values considering -2 and 2 as the origin
        if (des_quad_roll < -2.0 && des_quad_roll > -45.0)
        {
            des_quad_roll = des_quad_roll + 2.0;
        }
        if (des_quad_roll > 2.0 && des_quad_roll < 45.0)
        {
            des_quad_roll = des_quad_roll - 2.0;
        }

    // Scale the (-45 to 45) values to desired velocity limits
        des_quad_y_dot = des_quad_roll / slop_for_horizontal_scaling;

    // First ordered low pass filter
        des_quad_y_dot      = 0.686 * des_quad_y_dot_prev + 0.314 * des_quad_y_dot;
        des_quad_y_dot_prev = des_quad_y_dot;
        des_quad_y          = des_quad_y + (des_quad_y_dot + des_quad_y_dot_prev) / 2 * 0.0025;
        des_quad_y          = function_6_Bounds_on_XY_des(des_quad_y);

    ////////////////////////////////

    //////// Convert pilot_throttle into desired velocity in z
    // Trim (-20 to 20) data. This will help preventing the drone to drift
        if (pilot_throttle > -20.0 && pilot_throttle < 20.0)
        {
            pilot_throttle = 0.0;
        }
    // Start the values considering -2 and 2 as the origin
        if (pilot_throttle < -20.0 && pilot_throttle > -500.0)
        {
            pilot_throttle = pilot_throttle + 20.0;
        }
        if (pilot_throttle > 20.0 && pilot_throttle < 500.0)
        {
            pilot_throttle = pilot_throttle - 20.0;
        }
    // Scale the (-500 to 500) values to desired velocity limits
        des_quad_z_dot = pilot_throttle / slop_for_vertical_scaling;

    // First ordered low pass filter
        des_quad_z_dot          = 0.686 * des_quad_z_dot_prev + 0.314 * des_quad_z_dot;
        des_quad_z_dot_prev     = des_quad_z_dot;
        des_quad_z              = des_quad_z + (des_quad_z_dot + des_quad_z_dot_prev) / 2 * 0.0025;
        des_quad_z              = function_7_Bounds_on_Z_des(des_quad_z);

        if (des_quad_z < 0.1)
        {
            des_quad_z_dot = 0.0;
        }
        if (des_quad_z > 4.9)
        {
            des_quad_z_dot = 0.0;
        }

}

void ModeStabilize::function_2_Bydefault_Stabilize_code()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);

}

void ModeStabilize::function_3_Getting_Drone_States()
{
    // position in inertial frame of reference
    quad_x =   inertial_nav.get_position_neu_cm().x / 100.0 - quad_x_ini;         // m
    quad_y =  -inertial_nav.get_position_neu_cm().y / 100.0 - quad_y_ini;         // m
    quad_z =   inertial_nav.get_position_neu_cm().z / 100.0 - quad_z_ini;         // m

    // linear velocity in inertial frame of reference
    quad_x_dot =   inertial_nav.get_velocity_neu_cms().x /100.0;     // m/s
    quad_y_dot =  -inertial_nav.get_velocity_neu_cms().y /100.0;     // m/s
    quad_z_dot =   inertial_nav.get_velocity_neu_cms().z /100.0;     // m/s

    // hal.console->printf("Quad_Pos_Vel -> %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f \n", quad_x, quad_y, quad_z, quad_x_dot, quad_y_dot, quad_z_dot);

    quad_roll        =          (ahrs.roll_sensor)   / 100.0;     // degrees 
    quad_pitch       =        -(ahrs.pitch_sensor)   / 100.0;     // degrees 
    quad_yaw         =  360.0 -  (ahrs.yaw_sensor)   / 100.0;     // degrees 
    quad_roll_dot    =         (ahrs.get_gyro().x);               // degrees/second
    quad_pitch_dot   =        -(ahrs.get_gyro().y);               // degrees/second    
    quad_yaw_dot     =        -(ahrs.get_gyro().z);               // degrees/second

    // Get the current attitude of the drone

    Vector3f rpy_temp(quad_roll*PI/180.0,quad_pitch*PI/180.0,quad_yaw*PI/180.0);
    rpy         = rpy_temp;
    Matrix3f R_temp(math_function_1_eulerAnglesToRotationMatrix(rpy));
    R           = R_temp;
    Vector3f Omega_temp(quad_roll_dot*PI/180.0,quad_pitch_dot*PI/180.0,quad_yaw_dot*PI/180.0);
    Omega       = Omega_temp;

    // hal.console->printf("Quad_RPY -> %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f \n", quad_roll, quad_pitch, quad_yaw, quad_roll_dot, quad_pitch_dot, quad_yaw_dot);
}

void ModeStabilize::function_4_PWM_Generation(float F, float Mb1, float Mb2, float Mb3)
{
    float af    = 0.304;
    float aM    = 0.4948;
    float L     = 0.318198; 
    float c     = (af / aM)*(af / aM);
    
    PWM1_Custom = 1000 + 1000.0 * af * sqrtf( F/4.0 - Mb1 / (4.0 * L) - Mb2 / (4.0 * L) +  Mb3 / (4.0 * c) );
    PWM2_Custom = 1000 + 1000.0 * af * sqrtf( F/4.0 + Mb1 / (4.0 * L) + Mb2 / (4.0 * L) +  Mb3 / (4.0 * c) );
    PWM3_Custom = 1000 + 1000.0 * af * sqrtf( F/4.0 + Mb1 / (4.0 * L) - Mb2 / (4.0 * L) -  Mb3 / (4.0 * c) );
    PWM4_Custom = 1000 + 1000.0 * af * sqrtf( F/4.0 - Mb1 / (4.0 * L) + Mb2 / (4.0 * L) -  Mb3 / (4.0 * c) );

    if (PWM1_Custom < 1000){PWM1_Custom = 1000;}
    if (PWM2_Custom < 1000){PWM2_Custom = 1000;}
    if (PWM3_Custom < 1000){PWM3_Custom = 1000;}
    if (PWM4_Custom < 1000){PWM4_Custom = 1000;}

    if (PWM1_Custom > 2000){PWM1_Custom = 2000;}
    if (PWM2_Custom > 2000){PWM2_Custom = 2000;}
    if (PWM3_Custom > 2000){PWM3_Custom = 2000;}
    if (PWM4_Custom > 2000){PWM4_Custom = 2000;}
}

void ModeStabilize::function_5_Nonlinear_Geometric_Control()
{
    float F = 0;

    if (pilot_ch5 < 1250){

        // Get the desired attitude of the drone
        Vector3f rpy_d_local(des_quad_roll*PI/180.0,des_quad_pitch*PI/180.0,des_quad_yaw*PI/180.0);
        rpyd = rpy_d_local;

        Matrix3f Rd_local(math_function_1_eulerAnglesToRotationMatrix(rpyd));
        Rd = Rd_local;

        F = mq * g_acc + Kp_z * (des_quad_z - quad_z) + Kd_z * (des_quad_z_dot - quad_z_dot);
    }
    else if (pilot_ch5 > 1250 && pilot_ch5 < 1750)
    {
        Vector3f e_3_with_gravity(0, 0, g_acc);
        Vector3f u = (math_function_4_Matrix_vector_mul(Kxq, math_function_7_e_X()) + math_function_4_Matrix_vector_mul(Kxq_dot, math_function_8_e_X_dot()) + e_3_with_gravity);

        u[0] = mq * u[0];
        u[1] = mq * u[1];
        u[2] = mq * u[2];

        F = math_function_9_norm_of_vec(u);

        Vector3f b3d;
        b3d[0] = u[0] / math_function_9_norm_of_vec(u);
        b3d[1] = u[1] / math_function_9_norm_of_vec(u);
        b3d[2] = u[2] / math_function_9_norm_of_vec(u);
    
        Vector3f b1c(cosf(des_quad_yaw * PI / 180.0), sinf(des_quad_yaw * PI / 180.0), 0.0);
        Vector3f b2d;
        b2d = math_function_4_Matrix_vector_mul(math_function_10_hatmap(b3d), b1c);
        Vector3f b1d;
        b1d = math_function_4_Matrix_vector_mul(math_function_10_hatmap(b2d), b3d);
    
        Matrix3f Rd_temp_(b1d[0], b2d[0], b3d[0],
                          b1d[1], b2d[1], b3d[1],
                          b1d[2], b2d[2], b3d[2]);
        Rd = Rd_temp_;
    }

    // Compute attitude error
    Vector3f e_R_val        = math_function_5_e_R(R,Rd);
    Vector3f e_Omega_val    = math_function_6_e_Omega(R,Rd,Omega,Omegad);

    Vector3f M( math_function_4_Matrix_vector_mul(KR, e_R_val) + math_function_4_Matrix_vector_mul(KOmega,e_Omega_val) + Omega % math_function_4_Matrix_vector_mul(JJ, Omega));

    function_4_PWM_Generation(F, M[0], M[1], M[2]);

}

float ModeStabilize::function_6_Bounds_on_XY_des(float value)
{
    float Bounds_on_XY_des_value = 5.0;
    if (value > Bounds_on_XY_des_value)
    {
        value = Bounds_on_XY_des_value;
    }
    if (value < -Bounds_on_XY_des_value)
    {
        value = -Bounds_on_XY_des_value;
    }
    return value;
}

float ModeStabilize::function_7_Bounds_on_Z_des(float value)
{
    float Bounds_on_XY_des_value = 5.0;
    if (value > Bounds_on_XY_des_value)
    {
        value = Bounds_on_XY_des_value;
    }
    if (value < 0.0)
    {
        value = 0.0;
    }
    return value;
}

Matrix3f ModeStabilize::math_function_1_eulerAnglesToRotationMatrix(Vector3f rpy_local)
{

    // make sure the arguments of rpy_local[0], rpy_local[1], and rpy_local[2] are in radians.

    // Calculate rotation about x axis
   Matrix3f R_x (
              1,       0,              0,
              0,       cosf(rpy_local[0]),   -sinf(rpy_local[0]),
              0,       sinf(rpy_local[0]),   cosf(rpy_local[0])
              );

   // Calculate rotation about y axis
   Matrix3f R_y (
              cosf(rpy_local[1]),    0,      sinf(rpy_local[1]),
              0,               1,      0,
              -sinf(rpy_local[1]),   0,      cosf(rpy_local[1])
              );

   // Calculate rotation about z axis
   Matrix3f R_z (
              cosf(rpy_local[2]),    -sinf(rpy_local[2]),      0,
              sinf(rpy_local[2]),    cosf(rpy_local[2]),       0,
              0,               0,                  1);

   // Combined rotation matrix
   Matrix3f R_final = R_z * R_y * R_x;

   return R_final;
}

Matrix3f ModeStabilize::math_function_2_matrix_transpose(Matrix3f R_local)
{
    Matrix3f R_T(
            R_local[0][0],R_local[1][0],R_local[2][0],
            R_local[0][1],R_local[1][1],R_local[2][1],
            R_local[0][2],R_local[1][2],R_local[2][2]
            );
    return R_T;
}

Vector3f ModeStabilize::math_function_3_vee_map(Matrix3f R_local)
{
    Vector3f vector(R_local[2][1],R_local[0][2],R_local[1][0]);
    return vector;
}

Vector3f ModeStabilize::math_function_4_Matrix_vector_mul(Matrix3f R_local, Vector3f v)
{
    Vector3f mul_vector(
                        R_local[0][0]*v[0] + R_local[0][1]*v[1] + R_local[0][2]*v[2] ,
                        R_local[1][0]*v[0] + R_local[1][1]*v[1] + R_local[1][2]*v[2] ,
                        R_local[2][0]*v[0] + R_local[2][1]*v[1] + R_local[2][2]*v[2]
                        );
    return mul_vector;
}

Vector3f ModeStabilize::math_function_5_e_R(Matrix3f R_local, Matrix3f Rd_local)
{
    Vector3f error_vec(math_function_3_vee_map(math_function_2_matrix_transpose(Rd_local)*R_local - math_function_2_matrix_transpose(R_local)*Rd_local));
    return error_vec;
}

Vector3f ModeStabilize::math_function_6_e_Omega(Matrix3f R_local, Matrix3f Rd_local, Vector3f Omega_local, Vector3f Omegad_local)
{
    Vector3f error_vec(Omega_local - (math_function_2_matrix_transpose(R_local)*Rd_local)*Omegad_local);
    return error_vec;
}

Vector3f ModeStabilize::math_function_7_e_X()
{
    Vector3f err_pos_quad(des_quad_x - quad_x, des_quad_y - quad_y, des_quad_z - quad_z);
    return err_pos_quad;
}

Vector3f ModeStabilize::math_function_8_e_X_dot()
{
    Vector3f err_vel_quad(des_quad_x_dot - quad_x_dot, des_quad_y_dot - quad_y_dot, des_quad_z_dot - quad_z_dot);
    return err_vel_quad;
}

float ModeStabilize::math_function_9_norm_of_vec(Vector3f vec_)
{
    float norm_of_vec__ = sqrtf(vec_[0] * vec_[0] + vec_[1] * vec_[1] + vec_[2] * vec_[2]);
    return norm_of_vec__;
}

Matrix3f ModeStabilize::math_function_10_hatmap(Vector3f v)
{
    Matrix3f R_from_hatmap(
        0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0);
    return R_from_hatmap;
}

