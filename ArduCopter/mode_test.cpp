#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeTest::run()
{
    // apply simple mode transform to pilot inputs
    // update_simple_mode();

    // convert pilot input to lean angles
    // float target_roll_rad, target_pitch_rad;
    // get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

    // get pilot's desired yaw rate
    // float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } 
    // else if (copter.ap.throttle_zero
    //            || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
    //     // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
    //     // in order to facilitate the spoolup block

    //     // Attempting to Land
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    // } else {
    //     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // }

    // float pilot_desired_throttle = get_pilot_desired_throttle();

    // switch (motors->get_spool_state()) {
    // case AP_Motors::SpoolState::SHUT_DOWN:
        // // Motors Stopped
        // attitude_control->reset_yaw_target_and_rate();
        // attitude_control->reset_rate_controller_I_terms();
        // pilot_desired_throttle = 0.0f;
        // break;

    // case AP_Motors::SpoolState::GROUND_IDLE:
    //     // Landed
    //     attitude_control->reset_yaw_target_and_rate();
    //     attitude_control->reset_rate_controller_I_terms_smoothly();
    //     pilot_desired_throttle = 0.0f;
    //     break;

    // case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    //     // clear landing flag above zero throttle
    //     if (!motors->limit.throttle_lower) {
    //         set_land_complete(false);
    //     }
    //     break;

    // case AP_Motors::SpoolState::SPOOLING_UP:
    // case AP_Motors::SpoolState::SPOOLING_DOWN:
    //     // do nothing
    //     break;
    // }

    // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // output pilot's throttle
    // attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);

    srv_pos = sinf(AP_HAL::millis() / 1000.0f) * 0.1f;

    SRV_Channels::set_output_scaled(SRV_Channel::k_roll_out, srv_pos);
    SRV_Channels::calc_pwm();
    auto &srv = AP::srv();
    // cork now, so that all channel outputs happen at once
    srv.cork();
    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();
    flightmode->output_to_motors();
    // push all channels
    // motor output including servos and other updates that need to run at the main loop rate
    srv.push();
}
