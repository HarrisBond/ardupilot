/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Andy Piper          :Harmonic notch, In-flight FFT, Bi-directional DShot, various drivers
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel         :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland :PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

#include "harris_matrix.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller_main),
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    FAST_TASK(run_custom_controller),
#endif
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(heli_update_autorotation),
#endif //HELI_FRAME
    // send outputs to the motors library immediately
    // FAST_TASK(motors_output_main),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(update_heli_control_dynamics),
#endif //HELI_FRAME
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've landed or crashed
    FAST_TASK(update_land_and_crash_detectors),
    // surface tracking update
    FAST_TASK(update_rangefinder_terrain_offset),
#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &copter.camera_mount, update_fast),
#endif
#if HAL_LOGGING_ENABLED
    FAST_TASK(Log_Video_Stabilisation),
#endif

    SCHED_TASK(rc_loop,              250,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),
#if AP_FENCE_ENABLED
    SCHED_TASK(fence_check,           25,    100,  7),
#endif
    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &copter.optflow,             update,         200, 160,  12),
#endif
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK(update_batt_percent_for_leds, 10, 100, 16),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),
#if TOY_MODE_ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50,  24),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50,  27),
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    SCHED_TASK_CLASS(RC_Channels_Copter,   &copter.g2.rc_channels,      auto_trim_run,   10,  75,  30),
#endif
#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50,  36),
#endif
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50,  39),
#endif
    SCHED_TASK(update_altitude,       10,    100,  42),
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    SCHED_TASK(update_throttle_hover,100,     90,  48),
#if MODE_SMARTRTL_ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL,         &copter.mode_smartrtl,       save_position,    3, 100,  51),
#endif
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,               update,         3,  90,  54),
#endif
    SCHED_TASK(three_hz_loop,          3,     75, 57),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(update_precland,      400,     50,  69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75,  72),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(ekf_check,             10,     75,  84),
    SCHED_TASK(check_vibration,       10,     50,  87),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
    SCHED_TASK(takeoff_check,         50,     50,  91),
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landinggear_update,    10,     75,  93),
#endif
    SCHED_TASK(standby_update,        100,    75,  96),
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#endif
#if AP_TEMPCALIBRATION_ENABLED
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#endif
#if HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100, 138),
#endif  // HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    SCHED_TASK(afs_fs_check,          10,    100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK(terrain_update,        10,    100, 144),
#endif
#if AP_WINCH_ENABLED
    SCHED_TASK_CLASS(AP_Winch,             &copter.g2.winch,            update,          50,  50, 150),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // don't delete this, there is an equivalent (virtual) in AP_Vehicle for the non-rate loop case
    SCHED_TASK(update_dynamic_notch_at_specified_rate_main,                       LOOP_RATE, 200, 215),
#endif
};

void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];


#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if MODE_GUIDED_ENABLED
// set target location (for use by external control and scripting)
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

// start takeoff to given altitude (for use by scripting)
bool Copter::start_takeoff(const float alt_m)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start_m(alt_m)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}
#endif //MODE_GUIDED_ENABLED
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
#if MODE_GUIDED_ENABLED
// set target position (for use by scripting)
bool Copter::set_target_pos_NED(const Vector3f& target_pos_ned_m, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool is_terrain_alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3p pos_neu_m{target_pos_ned_m.x, target_pos_ned_m.y, -target_pos_ned_m.z};

    return mode_guided.set_pos_NEU_m(pos_neu_m, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), yaw_relative, is_terrain_alt);
}

// set target position and velocity (for use by scripting)
bool Copter::set_target_posvel_NED(const Vector3f& target_pos_ned_m, const Vector3f& target_vel_ned_ms)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3p pos_neu_m{target_pos_ned_m.x, target_pos_ned_m.y, -target_pos_ned_m.z};
    const Vector3f vel_neu_ms{target_vel_ned_ms.x, target_vel_ned_ms.y, -target_vel_ned_ms.z};

    return mode_guided.set_pos_vel_accel_NEU_m(pos_neu_m, vel_neu_ms, Vector3f());
}

// set target position, velocity and acceleration (for use by scripting)
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos_ned_m, const Vector3f& target_vel_ned_ms, const Vector3f& target_accel_ned_mss, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3p pos_neu_m{target_pos_ned_m.x, target_pos_ned_m.y, -target_pos_ned_m.z};
    const Vector3f vel_neu_ms{target_vel_ned_ms.x, target_vel_ned_ms.y, -target_vel_ned_ms.z};
    const Vector3f accel_neu_mss{target_accel_ned_mss.x, target_accel_ned_mss.y, -target_accel_ned_mss.z};

    return mode_guided.set_pos_vel_accel_NEU_m(pos_neu_m, vel_neu_ms, accel_neu_mss, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), yaw_relative);
}

bool Copter::set_target_velocity_NED(const Vector3f& target_vel_ned_ms)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f vel_neu_ms{target_vel_ned_ms.x, target_vel_ned_ms.y, -target_vel_ned_ms.z};
    mode_guided.set_vel_NEU_ms(vel_neu_ms);
    return true;
}

// set target velocity and acceleration (for use by scripting)
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel_ned_ms, const Vector3f& target_accel_ned_mss, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f vel_neu_ms{target_vel_ned_ms.x, target_vel_ned_ms.y, -target_vel_ned_ms.z};
    const Vector3f accel_neu_mss{target_accel_ned_mss.x, target_accel_ned_mss.y, -target_accel_ned_mss.z};

    mode_guided.set_vel_accel_NEU_m(vel_neu_ms, accel_neu_mss, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), relative_yaw);
    return true;
}

// set target roll pitch and yaw angles with throttle (for use by scripting)
bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    mode_guided.set_angle(q, Vector3f{}, climb_rate_ms*100, false);
    return true;
}

// set target roll pitch and yaw rates with throttle (for use by scripting)
bool Copter::set_target_rate_and_throttle(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps, float throttle)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // Zero quaternion indicates rate control
    Quaternion q;
    q.zero();

    // Convert from degrees per second to radians per second
    Vector3f ang_vel_body { roll_rate_dps, pitch_rate_dps, yaw_rate_dps };
    ang_vel_body *= DEG_TO_RAD;

    // Pass to guided mode
    mode_guided.set_angle(q, ang_vel_body, throttle, true);
    return true;
}

// Register a custom mode with given number and names
AP_Vehicle::custom_mode_state* Copter::register_custom_mode(const uint8_t num, const char* full_name, const char* short_name)
{
    const Mode::Number number = (Mode::Number)num;

    // See if this mode has been registered already, if it has return the state for it
    // This allows scripting restarts
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if (mode_guided_custom[i] == nullptr) {
            break;
        }
        if ((mode_guided_custom[i]->mode_number() == number) &&
            (strcmp(mode_guided_custom[i]->name(), full_name) == 0) &&
            (strncmp(mode_guided_custom[i]->name4(), short_name, 4) == 0)) {
            return &mode_guided_custom[i]->state;
        }
    }

    // Number already registered to existing mode
    if (mode_from_mode_num(number) != nullptr) {
        return nullptr;
    }

    // Find free slot
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if (mode_guided_custom[i] == nullptr) {
            // Duplicate strings so were not pointing to unknown memory
            const char* full_name_copy = strdup(full_name);
            const char* short_name_copy = strndup(short_name, 4);
            if ((full_name_copy != nullptr) && (short_name_copy != nullptr)) {
                mode_guided_custom[i] = NEW_NOTHROW ModeGuidedCustom(number, full_name_copy, short_name_copy);
            }
            if (mode_guided_custom[i] == nullptr) {
                // Allocation failure
                free((void*)full_name_copy);
                free((void*)short_name_copy);
                return nullptr;
            }

            // Registration successful, notify the GCS that it should re-request the available modes
            gcs().available_modes_changed();

            return &mode_guided_custom[i]->state;
        }
    }

    // No free slots
    return nullptr;
}
#endif // MODE_GUIDED_ENABLED

#if MODE_CIRCLE_ENABLED
// circle mode controls
bool Copter::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav->get_radius_m();
    return true;
}

bool Copter::set_circle_rate(float rate_degs)
{
    circle_nav->set_rate_degs(rate_degs);
    return true;
}
#endif

// set desired speed (m/s). Used for scripting.
bool Copter::set_desired_speed(float speed_ms)
{
    return flightmode->set_speed_NE_ms(speed_ms);
}

#if MODE_AUTO_ENABLED
// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Copter::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
#endif

// returns true if the EKF failsafe has triggered.  Only used by Lua scripts
bool Copter::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

// get target location (for use by scripting)
bool Copter::get_target_location(Location& target_loc)
{
    return flightmode->get_wp(target_loc);
}

/*
  update_target_location() acts as a wrapper for set_target_location
 */
bool Copter::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      by checking the caller has provided the correct old target
      location we prevent a race condition where the user changes mode
      or commands a different target in the controlling lua script
    */
    Location next_WP_loc;
    flightmode->get_wp(next_WP_loc);
    if (!old_loc.same_loc_as(next_WP_loc) ||
         old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }

    return set_target_location(new_loc);
}

#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing.
bool Copter::is_landing() const
{
    return flightmode->is_landing();
}

// returns true if vehicle is taking off.
bool Copter::is_taking_off() const
{
    return flightmode->is_taking_off();
}

bool Copter::current_mode_requires_mission() const
{
#if MODE_AUTO_ENABLED
        return flightmode == &mode_auto;
#else
        return false;
#endif
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

void Copter::update_batt_percent_for_leds(void){
    battery.read();
    float battery_percent = (battery.voltage() - 22.2) / (25.2 - 22.2);
    // Location loc;
    // ahrs.get_location(loc);
    // Vector3f pos;
    // if (!ahrs.get_relative_position_NED_origin_float(pos)) return;
    // Location::AltFrame alt_frame = loc.get_alt_frame();
    // float alt;
    // if (!loc.get_alt_m(alt_frame, alt)) return;
    // battery_percent = -alt;
    // battery_percent = pos.x;
    // Vector3 gyro_rate = ahrs.get_gyro();

    static float omega_zr = 994.87;
    static float omega_zr_dot = 0;

    Matrix<float, 13, 1> x;

    Vector3f world_pos;
    if (!ahrs.get_relative_position_NED_origin_float(world_pos)) return;
    x(0,0) = world_pos.x;
    x(1,0) = world_pos.y;
    x(2,0) = world_pos.z;

    Vector3f world_vel;
    if (!ahrs.get_velocity_NED(world_vel)) return;
    x(3,0) = world_vel.x;
    x(4,0) = -world_vel.y;
    x(5,0) = -world_vel.z;

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






    Matrix<float, 4, 13> K;

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

    Matrix<float, 1, 4> t_vec;
    t_vec(0, 0) = t*t*t, t_vec(0, 1) = t*t, t_vec(0, 2) = t, t_vec(0, 3) = 1;

    for (int r = 0; r < 4; r++){
        for (int c = 0; c < 13; c++){
            Matrix<float, 4, 1> p;
            p(0, 0) = K_0(r, c);p(1, 0) = K_1(r, c);p(2, 0) = K_2(r, c);p(3, 0) = K_3(r, c);
            Matrix<float, 1, 1> temp = t_vec * (M * p);
            K(r, c) = 0.5 * temp(0, 0);
        }
    }




    Matrix<float, 4, 1> u;
    u = K * x;

    omega_zr_dot = u(3, 0);

    battery_percent = x(0,0);

    notify.flags.battery_percent = battery_percent;
}

#if HAL_LOGGING_ENABLED
// Full rate logging of attitude, rate and pid loops
// should be run at loop rate
void Copter::loop_rate_logging()
{
   if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        if (!using_rate_thread) {
            Log_Write_Rate();
            Log_Write_PIDS(); // only logs if PIDS bitmask is set
        }
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_FTN_FAST) && !using_rate_thread) {
        AP::ins().write_notch_log_messages();
    }
#endif
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // always write AHRS attitude at 10Hz
    ahrs.Write_Attitude(attitude_control->get_att_target_euler_rad() * RAD_TO_DEG);
    // log attitude controller data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        if (!using_rate_thread) {
            Log_Write_Rate();
        }
    }
    if (!should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
    // log at 10Hz if PIDS bitmask is selected, even if no ATT bitmask is selected; logs at looprate if ATT_FAST and PIDS bitmask set
        if (!using_rate_thread) {
            Log_Write_PIDS();
        }
    }
    // log EKF attitude data always at 10Hz unless ATTITUDE_FAST, then do it in the 25Hz loop
    if (!should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if ((FRAME_CONFIG == HELI_FRAME) || should_log(MASK_LOG_MOTBATT)) {
        // always write motors log if we are a heli
        motors->Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
#if AP_RSSI_ENABLED
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
#endif
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
#if HAL_PROXIMITY_ENABLED
        g2.proximity.log();  // Write proximity sensor distances
#endif
#if AP_BEACON_ENABLED
        g2.beacon.log();
#endif
    }
#if AP_WINCH_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU();
    }

#if HAL_GYROFFT_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

// three_hz_loop - 3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

    // check for deadreckoning failsafe
    failsafe_deadreckon_check();

#if AP_RC_TRANSMITTER_TUNING_ENABLED
    //update transmitter based in flight tuning
    tuning();
#endif  // AP_RC_TRANSMITTER_TUNING_ENABLED

    // check if avoidance should be enabled based on alt
    low_alt_avoidance();
}

// ap_value calculates a 32-bit bitmask representing various pieces of
// state about the Copter.  It replaces a global variable which was
// used to track this state.
uint32_t Copter::ap_value() const
{
    uint32_t ret = 0;

    const bool *b = (const bool *)&ap;
    for (uint8_t i=0; i<sizeof(ap); i++) {
        if (b[i]) {
            ret |= 1U<<i;
        }
    }

    return ret;
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap_value());
    }
#endif

    if (!motors->armed()) {
        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->update_throttle_range();
#endif
    }

    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // log terrain data
    terrain_logging();
#endif

#if HAL_ADSB_ENABLED
    adsb.set_is_flying(!ap.land_complete);
#endif

    AP_Notify::flags.flying = !ap.land_complete;

    // slowly update the PID notches with the average loop rate
    if (!using_rate_thread) {
        attitude_control->set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    }
    pos_control->get_accel_U_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    custom_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#endif

#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // see if we should have a separate rate thread
    if (!started_rate_thread && get_fast_rate_type() != FastRateType::FAST_RATE_DISABLED) {
        if (hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Copter::rate_controller_thread, void),
                                         "rate",
                                         1536, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
            started_rate_thread = true;
        } else {
            AP_BoardConfig::allocation_error("rate thread");
        }
    }
#endif
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing_rad = wrap_2PI(ahrs.get_yaw_rad() + radians(180.0));
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

#if HAL_LOGGING_ENABLED
    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
#endif
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    // avoid processing bind-time RC values:
    if (!rc().has_valid_input()) {
        return;
    }

    if (simple_mode == SimpleMode::SIMPLE) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing_rad has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (simple_mode != SimpleMode::SUPERSIMPLE) {
            return;
        }
        if (home_distance_m() < SUPER_SIMPLE_RADIUS_M) {
            return;
        }
    }

    const float bearing_rad = home_bearing_rad();

    // check the bearing to home has changed by at least 5 degrees
    // todo: consider updating this continuously
    if (fabsf(wrap_PI(super_simple_last_bearing_rad - bearing_rad)) < radians(5.0)) {
        return;
    }

    super_simple_last_bearing_rad = bearing_rad;
    const float angle_rad = super_simple_last_bearing_rad + radians(180.0);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

void Copter::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in FAST_TASK.
    ahrs.update(true);
}

// read baro and log control tuning
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_FTN_FAST)) {
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
            AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages();
#endif
        }
    }
#endif
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance_m();
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing_deg();
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error_m() * 0.01;
    return true;
}

// get the target earth-frame angular velocities in rad/s (Z-axis component used by some gimbals)
bool Copter::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{
    // always returns zero vector if landed or disarmed
    if (copter.ap.land_complete) {
        rate_ef_targets.zero();
    } else {
        rate_ef_targets = attitude_control->get_rate_ef_targets();
    }
    return true;
}

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    :
    flight_modes(&g.flight_mode1),
    pos_variance_filt(FS_EKF_FILT_DEFAULT),
    vel_variance_filt(FS_EKF_FILT_DEFAULT),
    flightmode(&mode_stabilize),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    param_loader(var_info)
{
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
