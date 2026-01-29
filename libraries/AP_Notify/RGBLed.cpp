/*
   Generic RGBLed driver
*/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/


#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include "RGBLed.h"
#include "AP_Notify.h"
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Vehicle.h"
#include "Copter.h"

extern const AP_HAL::HAL& hal;

RGBLed::RGBLed(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim):
    _led_off(led_off),
    _led_bright(led_bright),
    _led_medium(led_medium),
    _led_dim(led_dim)
{

}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (red != _red_curr ||
        green != _green_curr ||
        blue != _blue_curr) {
        // call the hardware update routine
        if (hw_set_rgb(red, green, blue)) {
            _red_curr = red;
            _green_curr = green;
            _blue_curr = blue;
        }
    }
}

bool RGBLed::hw_set_rgb(uint8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    // default implementation ignores led id
    return hw_set_rgb(red, green, blue);
}

bool RGBLed::send_hw_rgb_changes()
{
    // default implementation does nothing
    return true;
}

RGBLed::Source RGBLed::rgb_source() const
{
    return Source(pNotify->_rgb_led_override.get());
}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (rgb_source() == Source::mavlink) {
        // don't set if in override mode
        return;
    }
    _set_rgb(red, green, blue);
}

uint8_t RGBLed::get_brightness(void) const
{
    uint8_t brightness = _led_bright;

    switch (pNotify->_rgb_led_brightness) {
    case RGB_LED_OFF:
        brightness = _led_off;
        break;
    case RGB_LED_LOW:
        brightness = _led_dim;
        break;
    case RGB_LED_MEDIUM:
        brightness = _led_medium;
        break;
    case RGB_LED_HIGH:
        brightness = _led_bright;
        break;
    }

    // use dim light when connected through USB
    if (hal.gpio->usb_connected() && brightness > _led_dim) {
        brightness = _led_dim;
    }
    return brightness;
}

uint32_t RGBLed::get_colour_sequence_obc(void) const
{
    if (AP_Notify::flags.armed) {
        return DEFINE_COLOUR_SEQUENCE_SOLID(RED);
    }
    return DEFINE_COLOUR_SEQUENCE_SOLID(GREEN);
}

// _scheduled_update - updates _red, _green, _blue according to notify flags
uint32_t RGBLed::get_colour_sequence(void) const
{
    // initialising pattern
    if (AP_Notify::flags.initialising) {
        return sequence_initialising;
    }

    // save trim or any calibration pattern
    if (AP_Notify::flags.save_trim ||
        AP_Notify::flags.esc_calibration ||
        AP_Notify::flags.compass_cal_running ||
        AP_Notify::flags.temp_cal_running) {
        return sequence_trim_or_esc;
    }

    // radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio ||
        AP_Notify::flags.failsafe_gcs ||
        AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.ekf_bad ||
        AP_Notify::flags.gps_glitching ||
        AP_Notify::flags.leak_detected) {

        if (AP_Notify::flags.leak_detected) {
            // purple if leak detected
            return sequence_failsafe_leak;
        } else if (AP_Notify::flags.ekf_bad) {
            // red on if ekf bad
            return sequence_failsafe_ekf;
        } else if (AP_Notify::flags.gps_glitching) {
            // blue on gps glitch
            return sequence_failsafe_gps_glitching;
        }
        // all off for radio or battery failsafe
        return sequence_failsafe_radio_or_battery;
    }

#if AP_GPS_ENABLED
    Location loc;
#if AP_AHRS_ENABLED
    // the AHRS can return "true" for get_location and still not be
    // happy enough with the location to set its origin from that
    // location:
    const bool good_ahrs_location = AP::ahrs().get_location(loc) && AP::ahrs().get_origin(loc);
#else
    const bool good_ahrs_location = true;
#endif

    // solid green or blue if armed
    if (AP_Notify::flags.armed) {
#if AP_GPS_ENABLED
        // solid green if armed with GPS 3d lock and good location
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D &&
            good_ahrs_location) {
            return sequence_armed;
        }
#endif
        // solid blue if armed with no GPS lock
        return sequence_armed_no_gps_or_no_location;
    }

    // double flash yellow if failing pre-arm checks
    if (!AP_Notify::flags.pre_arm_check) {
        return sequence_prearm_failing;
    }
    if (AP_Notify::flags.pre_arm_gps_check && good_ahrs_location) {
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS) {
            return sequence_disarmed_good_dgps_and_location;
        }
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            return sequence_disarmed_good_gps_and_location;
        }
    }
#endif

    return sequence_disarmed_bad_gps_or_no_location;
}

uint32_t RGBLed::get_colour_sequence_traffic_light(void) const
{
    if (AP_Notify::flags.initialising) {
        return DEFINE_COLOUR_SEQUENCE(RED,GREEN,BLUE,RED,GREEN,BLUE,RED,GREEN,BLUE,BLACK);
    }

    if (AP_Notify::flags.armed) {
        return DEFINE_COLOUR_SEQUENCE_SLOW(RED);
    }

    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        if (!AP_Notify::flags.pre_arm_check) {
            return DEFINE_COLOUR_SEQUENCE_ALTERNATE(YELLOW, BLACK);
        } else {
            return DEFINE_COLOUR_SEQUENCE_SLOW(YELLOW);
        }
    }

    if (!AP_Notify::flags.pre_arm_check) {
        return DEFINE_COLOUR_SEQUENCE_ALTERNATE(GREEN, BLACK);
    }
    return DEFINE_COLOUR_SEQUENCE_SLOW(GREEN);
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void RGBLed::update()
{
    typedef struct {
        double r;       // a fraction between 0 and 1
        double g;       // a fraction between 0 and 1
        double b;       // a fraction between 0 and 1
    } rgb;

    typedef struct {
        double h;       // angle in degrees
        double s;       // a fraction between 0 and 1
        double v;       // a fraction between 0 and 1
    } hsv;

    static hsv   rgb2hsv(rgb in);
    static rgb   hsv2rgb(hsv in);

    hsv rgb2hsv(rgb in)
    {
        hsv         out;
        double      min, max, delta;

        min = in.r < in.g ? in.r : in.g;
        min = min  < in.b ? min  : in.b;

        max = in.r > in.g ? in.r : in.g;
        max = max  > in.b ? max  : in.b;

        out.v = max;                                // v
        delta = max - min;
        if (delta < 0.00001)
        {
            out.s = 0;
            out.h = 0; // undefined, maybe nan?
            return out;
        }
        if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
            out.s = (delta / max);                  // s
        } else {
            // if max is 0, then r = g = b = 0              
            // s = 0, h is undefined
            out.s = 0.0;
            out.h = NAN;                            // its now undefined
            return out;
        }
        if( in.r >= max )                           // > is bogus, just keeps compilor happy
            out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
        else
        if( in.g >= max )
            out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

        out.h *= 60.0;                              // degrees

        if( out.h < 0.0 )
            out.h += 360.0;

        return out;
    }


    rgb hsv2rgb(hsv in)
    {
        double      hh, p, q, t, ff;
        long        i;
        rgb         out;

        if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
            out.r = in.v;
            out.g = in.v;
            out.b = in.v;
            return out;
        }
        hh = in.h;
        if(hh >= 360.0) hh = 0.0;
        hh /= 60.0;
        i = (long)hh;
        ff = hh - i;
        p = in.v * (1.0 - in.s);
        q = in.v * (1.0 - (in.s * ff));
        t = in.v * (1.0 - (in.s * (1.0 - ff)));

        switch(i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;

        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
        }
        return out;     
    }
    // Custom Code, Harris Bond
    bool custom_blink_test_enabled = true;
    if (custom_blink_test_enabled){
        static bool display_battery = true;
        float battery_voltage = -1.0;
        float battery_percent = -1.0;
        uint8_t battery_red;
        uint8_t battery_green;
        uint8_t battery_blue;

        if (display_battery){
            // read battery voltage and convert to a colour
            AP_Vehicle *vehicle = AP::vehicle.get_singleton();
            Copter *copter = dynamic_cast<Copter *>(vehicle);
            if (copter){
                //cast succeeded, vehicle is a copter
                //battery is read at 10Hz inside copter.cpp, so we dont need to call read() again here.
                battery_voltage = copter->battery.voltage();
                battery_percent = (battery_voltage - 22.2) / (25.2-22.2);
                hue_degrees = battery_percent * 120.0; // 0% = red (0 deg), 100% = green (120 deg)
                rgb battery_colour = hsv2rgb({hue_degrees, 1.0, 1.0});
                battery_red = (uint8_t)(battery_colour.r * 15.0);
                battery_green = (uint8_t)(battery_colour.g * 15.0);
                battery_blue = (uint8_t)(battery_colour.b * 15.0);
            }
        }

        // static bool toggle = false;
        // static uint32_t last_ms = 0;
        // if (AP_HAL::millis() - last_ms > 500){
        //     last_ms = AP_HAL::millis();
            // toggle = !toggle;
            // if (toggle){
            //     hw_set_rgb(1,15,0,15);
            // } else {
            //     hw_set_rgb(1,0,0,0);
            // }
        if (display_battery && battery_voltage > 0.0){
            set_rgb(battery_red, battery_green, battery_blue);
            return;
        }
        int front_led_ids[4] = {0, 1, 6, 7};
        int rear_led_ids[4] = {2, 3, 4, 5};
        for (int i = 0; i < sizeof(front_led_ids) / sizeof(front_led_ids[0]); i++){
            hw_set_rgb(front_led_ids[i], 15, 14, 17); // White
        }
        for (int i = 0; i < sizeof(rear_led_ids) / sizeof(rear_led_ids[0]); i++){
            hw_set_rgb(rear_led_ids[i], 15, 0, 0); // Red
        }
        send_hw_rgb_changes();
        // }
        return;
    }
    // End Custom Code

    uint32_t current_colour_sequence = 0;

    switch (rgb_source()) {
    case Source::mavlink:
        update_override();
        return; // note this is a return not a break!
    case Source::standard:
        current_colour_sequence = get_colour_sequence();
        break;
    case Source::obc:
        current_colour_sequence = get_colour_sequence_obc();
        break;
    case Source::traffic_light:
        current_colour_sequence = get_colour_sequence_traffic_light();
        break;
    }

    const uint8_t brightness = get_brightness();

    uint8_t step = (AP_HAL::millis()/100) % 10;

    // ensure we can't skip a step even with awful timing
    if (step != last_step) {
        step = (last_step+1) % 10;
        last_step = step;
    }

    const uint8_t colour = (current_colour_sequence >> (step*3)) & 7;

    uint8_t red_des = (colour & RED) ? brightness : _led_off;
    uint8_t green_des = (colour & GREEN) ? brightness : _led_off;
    uint8_t blue_des = (colour & BLUE) ? brightness : _led_off;

    set_rgb(red_des, green_des, blue_des);
}

#if AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED
/*
  handle LED control, only used when LED_OVERRIDE=1
*/
void RGBLed::handle_led_control(const mavlink_message_t &msg)
{
    if (rgb_source() != Source::mavlink) {
        // ignore LED_CONTROL commands if not in LED_OVERRIDE mode
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(&msg, &packet);

    _led_override.start_ms = AP_HAL::millis();

    uint8_t rate_hz = 0;
    switch (packet.custom_len) {
    case 4:
        rate_hz = packet.custom_bytes[3];
        FALLTHROUGH;
    case 3:
        rgb_control(packet.custom_bytes[0], packet.custom_bytes[1], packet.custom_bytes[2], rate_hz);
        break;
    }
}
#endif

/*
  update LED when in override mode
 */
void RGBLed::update_override(void)
{
    if (_led_override.rate_hz == 0) {
        // solid colour
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
        return;
    }
    // blinking
    uint32_t ms_per_cycle = 1000 / _led_override.rate_hz;
    uint32_t cycle = (AP_HAL::millis() - _led_override.start_ms) % ms_per_cycle;
    if (cycle > ms_per_cycle / 2) {
        // on
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
    } else {
        _set_rgb(0, 0, 0);
    }
}

/*
  RGB control
  give RGB and flash rate, used with scripting
*/
void RGBLed::rgb_control(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz)
{
    _led_override.rate_hz = rate_hz;
    _led_override.r = r;
    _led_override.g = g;
    _led_override.b = b;
}
