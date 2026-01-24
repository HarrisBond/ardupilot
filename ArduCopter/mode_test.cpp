#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
// void ModeTest::run()
// {
//     // apply simple mode transform to pilot inputs
//     // update_simple_mode();

//     // convert pilot input to lean angles
//     // float target_roll_rad, target_pitch_rad;
//     // get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

//     // get pilot's desired yaw rate
//     // float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

//     // if (!motors->armed()) {
//     //     // Motors should be Stopped
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//     // } 
//     // else if (copter.ap.throttle_zero
//     //            || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
//     //     // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
//     //     // in order to facilitate the spoolup block

//     //     // Attempting to Land
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
//     // } else {
//     //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//     // }

//     // float pilot_desired_throttle = get_pilot_desired_throttle();

//     // switch (motors->get_spool_state()) {
//     // case AP_Motors::SpoolState::SHUT_DOWN:
//         // // Motors Stopped
//         // attitude_control->reset_yaw_target_and_rate();
//         // attitude_control->reset_rate_controller_I_terms();
//         // pilot_desired_throttle = 0.0f;
//         // break;

//     // case AP_Motors::SpoolState::GROUND_IDLE:
//     //     // Landed
//     //     attitude_control->reset_yaw_target_and_rate();
//     //     attitude_control->reset_rate_controller_I_terms_smoothly();
//     //     pilot_desired_throttle = 0.0f;
//     //     break;

//     // case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//     //     // clear landing flag above zero throttle
//     //     if (!motors->limit.throttle_lower) {
//     //         set_land_complete(false);
//     //     }
//     //     break;

//     // case AP_Motors::SpoolState::SPOOLING_UP:
//     // case AP_Motors::SpoolState::SPOOLING_DOWN:
//     //     // do nothing
//     //     break;
//     // }

//     // call attitude controller
//     // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

//     // output pilot's throttle
//     // attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);

//     float srv_pos = sinf((float)(AP_HAL::millis()) / 1000.0f) * 0.1f;

    // SRV_Channels::set_output_scaled(SRV_Channel::k_roll_out, srv_pos);
//     SRV_Channels::calc_pwm();
//     auto &srv = AP::srv();
//     // cork now, so that all channel outputs happen at once
//     srv.cork();
//     // update output on any aux channels, for manual passthru
//     SRV_Channels::output_ch_all();
//     output_to_motors();
//     // push all channels
//     // motor output including servos and other updates that need to run at the main loop rate
//     srv.push();
// }










void ModeTest::run()
{

    static uint32_t start_time;
    static bool started = false;
    if (!started) {
        start_time = AP_HAL::millis();
        started = true;
    }

    for (uint8_t i=0; i<3; i++){

        SRV_Channel* ch = SRV_Channels::srv_channel(i);
    
        uint32_t elapsed_time_ms = AP_HAL::millis() - start_time;
    
        if (ch) {
            uint16_t pwm_us;
            // if (elapsed_time_ms < 20000){
            //     pwm_us = 1000;
            // } else {
            // pwm_us = 1050 + (int16_t)(sinf((float)AP_HAL::millis() / 1000.0f) * 50.0f);
            pwm_us = elapsed_time_ms % 2000 < 1000 ? 1400 : 1600;
            // }
            ch->set_output_pwm(pwm_us, true);   // force = true to ensure write
        }
    }

    // SRV_Channel* ch = SRV_Channels::srv_channel(3);
    // ch->set_output_pwm(1000, true);

    // ------------------------------------------
    // EDF thrust test: ramps from 1000 to 2000 in increments of 100.

    // SRV_Channel* ch = SRV_Channels::srv_channel(3);
    // uint32_t elapsed_time_ms = AP_HAL::millis() - start_time;
    // if (ch) {
    //     uint16_t pwm_us;
    //     uint32_t T = 20000;
    //     pwm_us = floor(elapsed_time_ms / T) * 100 + 1000;
    //     if (elapsed_time_ms % T > 0.5 * T || pwm_us > 1500){
    //         pwm_us = 1000;
    //     } 
    //     ch->set_output_pwm(pwm_us, true);   // force = true to ensure write
    // }

    // ------------------------------------------





    // float t = (float)AP_HAL::millis() * 0.001f;
    // float val = sinf(t) * 0.5f;  // normalized -1..1

    // SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, val);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, val);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, val);
    

    // convert any scaled outputs to pending PWM
    SRV_Channels::calc_pwm();

    // cork/push pair to synchronise outputs
    auto &srv = AP::srv();
    srv.cork();

    // write SRV_Channels into the HAL output buffers
    SRV_Channels::output_ch_all();

    // call the motor/ESC output routine so motor mixing / interlocks get handled
    motors->output();   // in Copter code `motors` is available (same as motors_output_main)

    // now push the buffered outputs to the HAL (actual pin update)
    srv.push();
}