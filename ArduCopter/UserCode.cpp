#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    // float pilot_roll      = channel_roll->get_control_in();         // -4500 to 4500
    // float pilot_pitch     = channel_pitch->get_control_in();        // -4500 to 4500
    // float pilot_yaw       = channel_yaw->get_control_in();          // -4500 to 4500
    // float pilot_throttle  = channel_throttle->get_control_in();     //     0 to 1000

    // hal.console->printf("Roll- %3.3f, Pitch- %3.3f, Yaw- %3.3f, Throttle- %3.3f, ", pilot_roll, pilot_pitch, pilot_yaw, pilot_throttle);

    // float pilot_ch5       = RC_Channels::get_radio_in(CH_5);        // 1000 to 2000
    // float pilot_ch6       = RC_Channels::get_radio_in(CH_6);        // 1000 to 2000
    // float pilot_ch7       = RC_Channels::get_radio_in(CH_7);        // 1000 to 2000
    // float pilot_ch8       = RC_Channels::get_radio_in(CH_8);        // 1000 to 2000

    // hal.console->printf("Ch5- %3.3f, Ch6- %3.3f, Ch7- %3.3f, Ch8- %3.3f\n", pilot_ch5, pilot_ch6, pilot_ch7, pilot_ch8);

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here

    // Custom_function();

}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::Custom_function(){

    // write your code here 
    hal.console->printf("Hello World from Custom Code \n");

  }