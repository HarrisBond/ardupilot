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





// void mulMat(int mat1[][C1], int mat2[][C2])
// {
//     int rslt[R1][C2];

//     for (int i = 0; i < R1; i++) {
//         for (int j = 0; j < C2; j++) {
//             rslt[i][j] = 0;

//             for (int k = 0; k < R2; k++) {
//                 rslt[i][j] += mat1[i][k] * mat2[k][j];
//             }
//         }
//     }

//     return rslt;
// }

void ModeTest::test_servos(uint32_t start_time)
{
    // Servo test: moves first 3 servos between 1400 and 1600 every second.
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
}

void ModeTest::set_servos(float alpha_0, float alpha_1, float alpha_2){
    Matrix<float, 3, 1> alpha;
    alpha(0, 0) = alpha_0;
    alpha(1, 0) = alpha_1;
    alpha(2, 0) = alpha_2;
    for (uint8_t i=0; i<3; i++){
        SRV_Channel* ch = SRV_Channels::srv_channel(i);
    
        if (ch) {
            uint16_t pwm_us = 1500 + (int16_t)(alpha(i, 0) * 10.0f);
            ch->set_output_pwm(pwm_us, true);   // force = true to ensure write
        }
    }
}

void ModeTest::neutralise_servos_and_edf()
{
    // set all servos to neutral position
    for (uint8_t i=0; i<3; i++){
        SRV_Channel* ch = SRV_Channels::srv_channel(i);
        if (ch){
            uint16_t pwm_us = 1500;
            ch->set_output_pwm(pwm_us, true);
        }
    }

    // set EDF throttle to zero
    SRV_Channel* ch = SRV_Channels::srv_channel(3);
    ch->set_output_pwm(1000, true);
}

void ModeTest::get_state_vector(Matrix<float, 13, 1>& x){
    Vector3f world_pos;
    if (!ahrs.get_relative_position_NED_origin_float(world_pos)) return;
    x(0,0) = world_pos.x;
    x(1,0) = world_pos.y;
    x(2,0) = world_pos.z;

    Vector3f world_vel;
    ahrs.get_velocity_NED(world_vel);
    x(3,0) = world_vel.x;
    x(4,0) = world_vel.y;
    x(5,0) = world_vel.z;

    // // note yaw and pitch are inverted here, because I use forward left up axes, but ardupilot uses forward right down.
    x(6,0) = -ahrs.get_yaw_rad();
    x(7,0) = -ahrs.get_pitch_rad();
    x(8,0) = ahrs.get_roll_rad();

    // gyro vector is roll pitch yaw, in rad/sec
    Vector3f angular_rate = ahrs.get_gyro();
    x(9,0) = -angular_rate.z;
    x(10,0) = -angular_rate.y;
    x(11,0) = angular_rate.x;

    static uint64_t last_time=0;
    omega_zr += omega_zr_dot * (AP_HAL::micros64() - last_time) / 1000000.0f;
    x(12,0) = omega_zr;
    last_time = AP_HAL::micros64();
}

void ModeTest::get_K(Matrix<float, 4, 13>& K, Matrix<float, 13, 1>& x){
    // cubic interpolation matrix
    float yaw = x(6, 0);

    static Matrix<float, 4, 4> M;
    static const float pi = 3.14159265358979323846f;
    M(0,0) = -1.0; M(0,1) = 3.0; M(0,2) = -3.0; M(0,3) = 1.0;
    M(1,0) = 2.0; M(1,1) = -5.0; M(1,2) = 4.0; M(1,3) = -1.0;
    M(2,0) = -1.0; M(2,1) = 0.0; M(2,2) = 1.0; M(2,3) = 0.0;
    M(3,0) = 0.0; M(3,1) = 2.0; M(3,2) = 0.0; M(3,3) = 0.0;

    const int num_K = 10;
    static Matrix<float, 4, 13> K_matrices[num_K];
    K_matrices[0](0, 0) = 2.33460; K_matrices[0](0, 1) = -1.10286; K_matrices[0](0, 2) = -0.24114; K_matrices[0](0, 3) = 2.71223; K_matrices[0](0, 4) = -1.27996; K_matrices[0](0, 5) = -0.72956; K_matrices[0](0, 6) = -1.82558; K_matrices[0](0, 7) = 7.51314; K_matrices[0](0, 8) = 4.47319; K_matrices[0](0, 9) = -0.81583; K_matrices[0](0, 10) = 0.12480; K_matrices[0](0, 11) = -1.23988; K_matrices[0](0, 12) = -0.01104; 
    K_matrices[0](1, 0) = -0.21219; K_matrices[0](1, 1) = 2.57325; K_matrices[0](1, 2) = -0.24114; K_matrices[0](1, 3) = -0.24540; K_matrices[0](1, 4) = 2.98884; K_matrices[0](1, 5) = -0.72956; K_matrices[0](1, 6) = -1.82558; K_matrices[0](1, 7) = 0.13040; K_matrices[0](1, 8) = -8.74317; K_matrices[0](1, 9) = -0.81583; K_matrices[0](1, 10) = -1.14336; K_matrices[0](1, 11) = 0.51291; K_matrices[0](1, 12) = -0.01104; 
    K_matrices[0](2, 0) = -2.12241; K_matrices[0](2, 1) = -1.47039; K_matrices[0](2, 2) = -0.24114; K_matrices[0](2, 3) = -2.46683; K_matrices[0](2, 4) = -1.70888; K_matrices[0](2, 5) = -0.72956; K_matrices[0](2, 6) = -1.82558; K_matrices[0](2, 7) = -7.64354; K_matrices[0](2, 8) = 4.26998; K_matrices[0](2, 9) = -0.81583; K_matrices[0](2, 10) = 1.01856; K_matrices[0](2, 11) = 0.72696; K_matrices[0](2, 12) = -0.01104; 
    K_matrices[0](3, 0) = 0.00000; K_matrices[0](3, 1) = 0.00000; K_matrices[0](3, 2) = 31.62002; K_matrices[0](3, 3) = 0.00000; K_matrices[0](3, 4) = 0.00000; K_matrices[0](3, 5) = 92.47258; K_matrices[0](3, 6) = -0.04177; K_matrices[0](3, 7) = 0.00000; K_matrices[0](3, 8) = -0.00000; K_matrices[0](3, 9) = -0.01445; K_matrices[0](3, 10) = -0.00000; K_matrices[0](3, 11) = -0.00000; K_matrices[0](3, 12) = 1.36850; 
    K_matrices[1](0, 0) = 1.13117; K_matrices[1](0, 1) = -2.32102; K_matrices[1](0, 2) = -0.24114; K_matrices[1](0, 3) = 1.26109; K_matrices[1](0, 4) = -2.58503; K_matrices[1](0, 5) = -0.72956; K_matrices[1](0, 6) = -1.82558; K_matrices[1](0, 7) = 2.93963; K_matrices[1](0, 8) = 7.46713; K_matrices[1](0, 9) = -0.81583; K_matrices[1](0, 10) = 0.05804; K_matrices[1](0, 11) = -1.18111; K_matrices[1](0, 12) = -0.01104; 
    K_matrices[1](1, 0) = 1.44447; K_matrices[1](1, 1) = 2.14013; K_matrices[1](1, 2) = -0.24114; K_matrices[1](1, 3) = 1.60975; K_matrices[1](1, 4) = 2.38326; K_matrices[1](1, 5) = -0.72956; K_matrices[1](1, 6) = -1.82558; K_matrices[1](1, 7) = 5.00587; K_matrices[1](1, 8) = -6.27158; K_matrices[1](1, 9) = -0.81583; K_matrices[1](1, 10) = -1.05861; K_matrices[1](1, 11) = 0.54087; K_matrices[1](1, 12) = -0.01104; 
    K_matrices[1](2, 0) = -2.57564; K_matrices[1](2, 1) = 0.18089; K_matrices[1](2, 2) = -0.24114; K_matrices[1](2, 3) = -2.87084; K_matrices[1](2, 4) = 0.20177; K_matrices[1](2, 5) = -0.72956; K_matrices[1](2, 6) = -1.82558; K_matrices[1](2, 7) = -7.94550; K_matrices[1](2, 8) = -1.19554; K_matrices[1](2, 9) = -0.81583; K_matrices[1](2, 10) = 1.00057; K_matrices[1](2, 11) = 0.64025; K_matrices[1](2, 12) = -0.01104; 
    K_matrices[1](3, 0) = -0.00000; K_matrices[1](3, 1) = 0.00000; K_matrices[1](3, 2) = 31.62002; K_matrices[1](3, 3) = 0.00000; K_matrices[1](3, 4) = 0.00000; K_matrices[1](3, 5) = 92.47258; K_matrices[1](3, 6) = -0.04177; K_matrices[1](3, 7) = 0.00000; K_matrices[1](3, 8) = -0.00000; K_matrices[1](3, 9) = -0.01445; K_matrices[1](3, 10) = -0.00000; K_matrices[1](3, 11) = 0.00000; K_matrices[1](3, 12) = 1.36850; 
    K_matrices[2](0, 0) = -0.48604; K_matrices[2](0, 1) = -2.53583; K_matrices[2](0, 2) = -0.24114; K_matrices[2](0, 3) = -0.53076; K_matrices[2](0, 4) = -2.77522; K_matrices[2](0, 5) = -0.72956; K_matrices[2](0, 6) = -1.82558; K_matrices[2](0, 7) = -1.82065; K_matrices[2](0, 8) = 7.50986; K_matrices[2](0, 9) = -0.81583; K_matrices[2](0, 10) = -0.02441; K_matrices[2](0, 11) = -1.17354; K_matrices[2](0, 12) = -0.01104; 
    K_matrices[2](1, 0) = 2.43911; K_matrices[2](1, 1) = 0.84700; K_matrices[2](1, 2) = -0.24114; K_matrices[2](1, 3) = 2.66910; K_matrices[2](1, 4) = 0.92589; K_matrices[2](1, 5) = -0.72956; K_matrices[2](1, 6) = -1.82558; K_matrices[2](1, 7) = 7.41577; K_matrices[2](1, 8) = -2.16680; K_matrices[2](1, 9) = -0.81583; K_matrices[2](1, 10) = -1.01084; K_matrices[2](1, 11) = 0.60782; K_matrices[2](1, 12) = -0.01104; 
    K_matrices[2](2, 0) = -1.95308; K_matrices[2](2, 1) = 1.68883; K_matrices[2](2, 2) = -0.24114; K_matrices[2](2, 3) = -2.13834; K_matrices[2](2, 4) = 1.84933; K_matrices[2](2, 5) = -0.72956; K_matrices[2](2, 6) = -1.82558; K_matrices[2](2, 7) = -5.59512; K_matrices[2](2, 8) = -5.34307; K_matrices[2](2, 9) = -0.81583; K_matrices[2](2, 10) = 1.03524; K_matrices[2](2, 11) = 0.56572; K_matrices[2](2, 12) = -0.01104; 
    K_matrices[2](3, 0) = -0.00000; K_matrices[2](3, 1) = -0.00000; K_matrices[2](3, 2) = 31.62002; K_matrices[2](3, 3) = 0.00000; K_matrices[2](3, 4) = -0.00000; K_matrices[2](3, 5) = 92.47258; K_matrices[2](3, 6) = -0.04177; K_matrices[2](3, 7) = 0.00000; K_matrices[2](3, 8) = 0.00000; K_matrices[2](3, 9) = -0.01445; K_matrices[2](3, 10) = -0.00000; K_matrices[2](3, 11) = -0.00000; K_matrices[2](3, 12) = 1.36850; 
    K_matrices[3](0, 0) = -1.89738; K_matrices[3](0, 1) = -1.75118; K_matrices[3](0, 2) = -0.24114; K_matrices[3](0, 3) = -2.10113; K_matrices[3](0, 4) = -1.94087; K_matrices[3](0, 5) = -0.72956; K_matrices[3](0, 6) = -1.82558; K_matrices[3](0, 7) = -5.91469; K_matrices[3](0, 8) = 5.25653; K_matrices[3](0, 9) = -0.81583; K_matrices[3](0, 10) = -0.09667; K_matrices[3](0, 11) = -1.21170; K_matrices[3](0, 12) = -0.01104; 
    K_matrices[3](1, 0) = 2.46525; K_matrices[3](1, 1) = -0.76759; K_matrices[3](1, 2) = -0.24114; K_matrices[3](1, 3) = 2.73028; K_matrices[3](1, 4) = -0.85105; K_matrices[3](1, 5) = -0.72956; K_matrices[3](1, 6) = -1.82558; K_matrices[3](1, 7) = 7.50335; K_matrices[3](1, 8) = 2.50435; K_matrices[3](1, 9) = -0.81583; K_matrices[3](1, 10) = -1.00818; K_matrices[3](1, 11) = 0.68886; K_matrices[3](1, 12) = -0.01104; 
    K_matrices[3](2, 0) = -0.56787; K_matrices[3](2, 1) = 2.51877; K_matrices[3](2, 2) = -0.24114; K_matrices[3](2, 3) = -0.62915; K_matrices[3](2, 4) = 2.79192; K_matrices[3](2, 5) = -0.72956; K_matrices[3](2, 6) = -1.82558; K_matrices[3](2, 7) = -1.58867; K_matrices[3](2, 8) = -7.76088; K_matrices[3](2, 9) = -0.81583; K_matrices[3](2, 10) = 1.10484; K_matrices[3](2, 11) = 0.52284; K_matrices[3](2, 12) = -0.01104; 
    K_matrices[3](3, 0) = -0.00000; K_matrices[3](3, 1) = 0.00000; K_matrices[3](3, 2) = 31.62002; K_matrices[3](3, 3) = -0.00000; K_matrices[3](3, 4) = 0.00000; K_matrices[3](3, 5) = 92.47258; K_matrices[3](3, 6) = -0.04177; K_matrices[3](3, 7) = -0.00000; K_matrices[3](3, 8) = -0.00000; K_matrices[3](3, 9) = -0.01445; K_matrices[3](3, 10) = 0.00000; K_matrices[3](3, 11) = 0.00000; K_matrices[3](3, 12) = 1.36850; 
    K_matrices[4](0, 0) = -2.57209; K_matrices[4](0, 1) = -0.22590; K_matrices[4](0, 2) = -0.24114; K_matrices[4](0, 3) = -2.95730; K_matrices[4](0, 4) = -0.26103; K_matrices[4](0, 5) = -0.72956; K_matrices[4](0, 6) = -1.82558; K_matrices[4](0, 7) = -8.49572; K_matrices[4](0, 8) = 0.73836; K_matrices[4](0, 9) = -0.81583; K_matrices[4](0, 10) = -0.13607; K_matrices[4](0, 11) = -1.28773; K_matrices[4](0, 12) = -0.01104; 
    K_matrices[4](1, 0) = 1.48168; K_matrices[4](1, 1) = -2.11454; K_matrices[4](1, 2) = -0.24114; K_matrices[4](1, 3) = 1.70253; K_matrices[4](1, 4) = -2.43136; K_matrices[4](1, 5) = -0.72956; K_matrices[4](1, 6) = -1.82558; K_matrices[4](1, 7) = 4.87468; K_matrices[4](1, 8) = 6.99280; K_matrices[4](1, 9) = -0.81583; K_matrices[4](1, 10) = -1.05504; K_matrices[4](1, 11) = 0.76060; K_matrices[4](1, 12) = -0.01104; 
    K_matrices[4](2, 0) = 1.09041; K_matrices[4](2, 1) = 2.34044; K_matrices[4](2, 2) = -0.24114; K_matrices[4](2, 3) = 1.25477; K_matrices[4](2, 4) = 2.69239; K_matrices[4](2, 5) = -0.72956; K_matrices[4](2, 6) = -1.82558; K_matrices[4](2, 7) = 3.62104; K_matrices[4](2, 8) = -7.73117; K_matrices[4](2, 9) = -0.81583; K_matrices[4](2, 10) = 1.19111; K_matrices[4](2, 11) = 0.52713; K_matrices[4](2, 12) = -0.01104; 
    K_matrices[4](3, 0) = 0.00000; K_matrices[4](3, 1) = -0.00000; K_matrices[4](3, 2) = 31.62002; K_matrices[4](3, 3) = 0.00000; K_matrices[4](3, 4) = -0.00000; K_matrices[4](3, 5) = 92.47258; K_matrices[4](3, 6) = -0.04177; K_matrices[4](3, 7) = 0.00000; K_matrices[4](3, 8) = 0.00000; K_matrices[4](3, 9) = -0.01445; K_matrices[4](3, 10) = -0.00000; K_matrices[4](3, 11) = 0.00000; K_matrices[4](3, 12) = 1.36850; 
    K_matrices[5](0, 0) = -2.11554; K_matrices[5](0, 1) = 1.48025; K_matrices[5](0, 2) = -0.24114; K_matrices[5](0, 3) = -2.55029; K_matrices[5](0, 4) = 1.78278; K_matrices[5](0, 5) = -0.72956; K_matrices[5](0, 6) = -1.82558; K_matrices[5](0, 7) = -7.66196; K_matrices[5](0, 8) = -5.40154; K_matrices[5](0, 9) = -0.81583; K_matrices[5](0, 10) = -0.11736; K_matrices[5](0, 11) = -1.38085; K_matrices[5](0, 12) = -0.01104; 
    K_matrices[5](1, 0) = -0.22417; K_matrices[5](1, 1) = -2.57224; K_matrices[5](1, 2) = -0.24114; K_matrices[5](1, 3) = -0.27111; K_matrices[5](1, 4) = -3.09919; K_matrices[5](1, 5) = -0.72956; K_matrices[5](1, 6) = -1.82558; K_matrices[5](1, 7) = -0.86098; K_matrices[5](1, 8) = 9.33122; K_matrices[5](1, 9) = -0.81583; K_matrices[5](1, 10) = -1.14585; K_matrices[5](1, 11) = 0.79102; K_matrices[5](1, 12) = -0.01104; 
    K_matrices[5](2, 0) = 2.33971; K_matrices[5](2, 1) = 1.09198; K_matrices[5](2, 2) = -0.24114; K_matrices[5](2, 3) = 2.82140; K_matrices[5](2, 4) = 1.31640; K_matrices[5](2, 5) = -0.72956; K_matrices[5](2, 6) = -1.82558; K_matrices[5](2, 7) = 8.52294; K_matrices[5](2, 8) = -3.92968; K_matrices[5](2, 9) = -0.81583; K_matrices[5](2, 10) = 1.26321; K_matrices[5](2, 11) = 0.58983; K_matrices[5](2, 12) = -0.01104; 
    K_matrices[5](3, 0) = -0.00000; K_matrices[5](3, 1) = 0.00000; K_matrices[5](3, 2) = 31.62002; K_matrices[5](3, 3) = -0.00000; K_matrices[5](3, 4) = 0.00000; K_matrices[5](3, 5) = 92.47258; K_matrices[5](3, 6) = -0.04177; K_matrices[5](3, 7) = -0.00000; K_matrices[5](3, 8) = -0.00000; K_matrices[5](3, 9) = -0.01445; K_matrices[5](3, 10) = -0.00000; K_matrices[5](3, 11) = -0.00000; K_matrices[5](3, 12) = 1.36850; 
    K_matrices[6](0, 0) = -0.56787; K_matrices[6](0, 1) = 2.51877; K_matrices[6](0, 2) = -0.24114; K_matrices[6](0, 3) = -0.71001; K_matrices[6](0, 4) = 3.14285; K_matrices[6](0, 5) = -0.72956; K_matrices[6](0, 6) = -1.82558; K_matrices[6](0, 7) = -1.94078; K_matrices[6](0, 8) = -9.87111; K_matrices[6](0, 9) = -0.81583; K_matrices[6](0, 10) = -0.03269; K_matrices[6](0, 11) = -1.44333; K_matrices[6](0, 12) = -0.01104; 
    K_matrices[6](1, 0) = -1.89738; K_matrices[6](1, 1) = -1.75118; K_matrices[6](1, 2) = -0.24114; K_matrices[6](1, 3) = -2.36810; K_matrices[6](1, 4) = -2.18413; K_matrices[6](1, 5) = -0.72956; K_matrices[6](1, 6) = -1.82558; K_matrices[6](1, 7) = -7.58652; K_matrices[6](1, 8) = 6.60258; K_matrices[6](1, 9) = -0.81583; K_matrices[6](1, 10) = -1.24278; K_matrices[6](1, 11) = 0.74957; K_matrices[6](1, 12) = -0.01104; 
    K_matrices[6](2, 0) = 2.46525; K_matrices[6](2, 1) = -0.76759; K_matrices[6](2, 2) = -0.24114; K_matrices[6](2, 3) = 3.07811; K_matrices[6](2, 4) = -0.95872; K_matrices[6](2, 5) = -0.72956; K_matrices[6](2, 6) = -1.82558; K_matrices[6](2, 7) = 9.52730; K_matrices[6](2, 8) = 3.26853; K_matrices[6](2, 9) = -0.81583; K_matrices[6](2, 10) = 1.27547; K_matrices[6](2, 11) = 0.69376; K_matrices[6](2, 12) = -0.01104; 
    K_matrices[6](3, 0) = -0.00000; K_matrices[6](3, 1) = -0.00000; K_matrices[6](3, 2) = 31.62002; K_matrices[6](3, 3) = -0.00000; K_matrices[6](3, 4) = 0.00000; K_matrices[6](3, 5) = 92.47258; K_matrices[6](3, 6) = -0.04177; K_matrices[6](3, 7) = -0.00000; K_matrices[6](3, 8) = -0.00000; K_matrices[6](3, 9) = -0.01445; K_matrices[6](3, 10) = -0.00000; K_matrices[6](3, 11) = 0.00000; K_matrices[6](3, 12) = 1.36850; 
    K_matrices[7](0, 0) = 1.32403; K_matrices[7](0, 1) = 2.21667; K_matrices[7](0, 2) = -0.24114; K_matrices[7](0, 3) = 1.65865; K_matrices[7](0, 4) = 2.77951; K_matrices[7](0, 5) = -0.72956; K_matrices[7](0, 6) = -1.82558; K_matrices[7](0, 7) = 5.74880; K_matrices[7](0, 8) = -8.38790; K_matrices[7](0, 9) = -0.81583; K_matrices[7](0, 10) = 0.07631; K_matrices[7](0, 11) = -1.42662; K_matrices[7](0, 12) = -0.01104; 
    K_matrices[7](1, 0) = -2.58170; K_matrices[7](1, 1) = 0.03831; K_matrices[7](1, 2) = -0.24114; K_matrices[7](1, 3) = -3.23607; K_matrices[7](1, 4) = 0.04917; K_matrices[7](1, 5) = -0.72956; K_matrices[7](1, 6) = -1.82558; K_matrices[7](1, 7) = -10.13610; K_matrices[7](1, 8) = -0.80044; K_matrices[7](1, 9) = -0.81583; K_matrices[7](1, 10) = -1.28257; K_matrices[7](1, 11) = 0.64773; K_matrices[7](1, 12) = -0.01104; 
    K_matrices[7](2, 0) = 1.25768; K_matrices[7](2, 1) = -2.25498; K_matrices[7](2, 2) = -0.24114; K_matrices[7](2, 3) = 1.57742; K_matrices[7](2, 4) = -2.82869; K_matrices[7](2, 5) = -0.72956; K_matrices[7](2, 6) = -1.82558; K_matrices[7](2, 7) = 4.38730; K_matrices[7](2, 8) = 9.18834; K_matrices[7](2, 9) = -0.81583; K_matrices[7](2, 10) = 1.20626; K_matrices[7](2, 11) = 0.77889; K_matrices[7](2, 12) = -0.01104; 
    K_matrices[7](3, 0) = -0.00000; K_matrices[7](3, 1) = -0.00000; K_matrices[7](3, 2) = 31.62002; K_matrices[7](3, 3) = -0.00000; K_matrices[7](3, 4) = -0.00000; K_matrices[7](3, 5) = 92.47258; K_matrices[7](3, 6) = -0.04177; K_matrices[7](3, 7) = -0.00000; K_matrices[7](3, 8) = 0.00000; K_matrices[7](3, 9) = -0.01445; K_matrices[7](3, 10) = -0.00000; K_matrices[7](3, 11) = 0.00000; K_matrices[7](3, 12) = 1.36850; 
    K_matrices[8](0, 0) = 2.48218; K_matrices[8](0, 1) = 0.71095; K_matrices[8](0, 2) = -0.24114; K_matrices[8](0, 3) = 3.02276; K_matrices[8](0, 4) = 0.86709; K_matrices[8](0, 5) = -0.72956; K_matrices[8](0, 6) = -1.82558; K_matrices[8](0, 7) = 9.44344; K_matrices[8](0, 8) = -1.80687; K_matrices[8](0, 9) = -0.81583; K_matrices[8](0, 10) = 0.13908; K_matrices[8](0, 11) = -1.33839; K_matrices[8](0, 12) = -0.01104; 
    K_matrices[8](1, 0) = -1.85679; K_matrices[8](1, 1) = 1.79416; K_matrices[8](1, 2) = -0.24114; K_matrices[8](1, 3) = -2.26048; K_matrices[8](1, 4) = 2.18581; K_matrices[8](1, 5) = -0.72956; K_matrices[8](1, 6) = -1.82558; K_matrices[8](1, 7) = -6.27537; K_matrices[8](1, 8) = -7.28445; K_matrices[8](1, 9) = -0.81583; K_matrices[8](1, 10) = -1.23670; K_matrices[8](1, 11) = 0.54984; K_matrices[8](1, 12) = -0.01104; 
    K_matrices[8](2, 0) = -0.62539; K_matrices[8](2, 1) = -2.50510; K_matrices[8](2, 2) = -0.24114; K_matrices[8](2, 3) = -0.76228; K_matrices[8](2, 4) = -3.05290; K_matrices[8](2, 5) = -0.72956; K_matrices[8](2, 6) = -1.82558; K_matrices[8](2, 7) = -3.16806; K_matrices[8](2, 8) = 9.09132; K_matrices[8](2, 9) = -0.81583; K_matrices[8](2, 10) = 1.09762; K_matrices[8](2, 11) = 0.78856; K_matrices[8](2, 12) = -0.01104; 
    K_matrices[8](3, 0) = -0.00000; K_matrices[8](3, 1) = 0.00000; K_matrices[8](3, 2) = 31.62002; K_matrices[8](3, 3) = -0.00000; K_matrices[8](3, 4) = 0.00000; K_matrices[8](3, 5) = 92.47258; K_matrices[8](3, 6) = -0.04177; K_matrices[8](3, 7) = -0.00000; K_matrices[8](3, 8) = -0.00000; K_matrices[8](3, 9) = -0.01445; K_matrices[8](3, 10) = 0.00000; K_matrices[8](3, 11) = 0.00000; K_matrices[8](3, 12) = 1.36850; 
    K_matrices[9](0, 0) = 2.33460; K_matrices[9](0, 1) = -1.10286; K_matrices[9](0, 2) = -0.24114; K_matrices[9](0, 3) = 2.71223; K_matrices[9](0, 4) = -1.27996; K_matrices[9](0, 5) = -0.72956; K_matrices[9](0, 6) = -1.82558; K_matrices[9](0, 7) = 7.51314; K_matrices[9](0, 8) = 4.47319; K_matrices[9](0, 9) = -0.81583; K_matrices[9](0, 10) = 0.12480; K_matrices[9](0, 11) = -1.23988; K_matrices[9](0, 12) = -0.01104; 
    K_matrices[9](1, 0) = -0.21219; K_matrices[9](1, 1) = 2.57325; K_matrices[9](1, 2) = -0.24114; K_matrices[9](1, 3) = -0.24540; K_matrices[9](1, 4) = 2.98884; K_matrices[9](1, 5) = -0.72956; K_matrices[9](1, 6) = -1.82558; K_matrices[9](1, 7) = 0.13040; K_matrices[9](1, 8) = -8.74317; K_matrices[9](1, 9) = -0.81583; K_matrices[9](1, 10) = -1.14336; K_matrices[9](1, 11) = 0.51291; K_matrices[9](1, 12) = -0.01104; 
    K_matrices[9](2, 0) = -2.12241; K_matrices[9](2, 1) = -1.47039; K_matrices[9](2, 2) = -0.24114; K_matrices[9](2, 3) = -2.46683; K_matrices[9](2, 4) = -1.70888; K_matrices[9](2, 5) = -0.72956; K_matrices[9](2, 6) = -1.82558; K_matrices[9](2, 7) = -7.64354; K_matrices[9](2, 8) = 4.26998; K_matrices[9](2, 9) = -0.81583; K_matrices[9](2, 10) = 1.01856; K_matrices[9](2, 11) = 0.72696; K_matrices[9](2, 12) = -0.01104; 
    K_matrices[9](3, 0) = -0.00000; K_matrices[9](3, 1) = 0.00000; K_matrices[9](3, 2) = 31.62002; K_matrices[9](3, 3) = -0.00000; K_matrices[9](3, 4) = 0.00000; K_matrices[9](3, 5) = 92.47258; K_matrices[9](3, 6) = -0.04177; K_matrices[9](3, 7) = -0.00000; K_matrices[9](3, 8) = -0.00000; K_matrices[9](3, 9) = -0.01445; K_matrices[9](3, 10) = 0.00000; K_matrices[9](3, 11) = 0.00000; K_matrices[9](3, 12) = 1.36850; 
    

    float i = (yaw + pi) * (num_K-1) / (2 * pi);
    float t = (i - floor(i));
    const float eps = 1e-5;
    if (t < eps){
        i += eps;
        t += eps;
    }
    int i_0 = (int(floor(i) - 1) % (num_K));int i_1 = (int(floor(i)) % (num_K));int i_2 = (int(ceil(i)) % (num_K));int i_3 = (int(ceil(i) + 1) % (num_K));

    Matrix<float, 4, 13> K_0 = K_matrices[i_0];Matrix<float, 4, 13> K_1 = K_matrices[i_1];Matrix<float, 4, 13> K_2 = K_matrices[i_2];Matrix<float, 4, 13> K_3 = K_matrices[i_3];

    Matrix<float, 4, 1> t_vec;
    t_vec(0, 0) = 1;t_vec(1, 0) = t;t_vec(2, 0) = t * t;t_vec(3, 0) = t * t * t;

    for (int r = 0; r < 4; r++){
        for (int c = 0; c < 13; c++){
            Matrix<float, 4, 1> p;
            p(0, 0) = K_0(r, c);p(1, 0) = K_1(r, c);p(2, 0) = K_2(r, c);p(3, 0) = K_3(r, c);
            K(r, c) = 0.5 * t_vec * M * p;
        }
    }
}

void ModeTest::run()
{
    static uint32_t start_time;
    static bool started = false;
    if (!started) {
        start_time = AP_HAL::millis();
        started = true;
    }

    if (motors->armed()){
        
        Matrix<float, 2, 3> A;
        Matrix<float, 3, 2> B;

        // Fill manually
        A(0,0)=1; A(0,1)=2; A(0,2)=3;
        A(1,0)=4; A(1,1)=5; A(1,2)=6;

        B(0,0)=7;  B(0,1)=8;
        B(1,0)=9;  B(1,1)=10;
        B(2,0)=11; B(2,1)=12;

        // Matrix multiply
        auto C = A * B;

        // Vector multiply
        float v[3] = {1,2,3};
        float r[2];
        A.multiply(v, r);

        Matrix<float, 13, 1> x;
        get_state_vector(x);

        Matrix<float, 4, 13> K;
        get_K(K, x);

        Matrix<float, 4, 1> u;
        u = K * x;

        set_servos(u(0, 0), u(1, 0), u(2, 0), u(3, 0));

        omega_zr_dot = u(3, 0);


        // test_servos(start_time);
    } else {
        neutralise_servos_and_edf();
    }
    
    
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