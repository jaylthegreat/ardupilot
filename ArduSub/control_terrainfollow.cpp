#include "Sub.h"

/*
 * control_althold.pde - init and run calls for althold, flight mode
 */
const int samples = 6;
uint16_t last_valid_rngfnd_alt;
uint16_t rngfnd_alts [samples];
uint16_t baro_alts [samples];

int count;
bool near_bottom; // keep track if you're within 10m of bottom and can trust rangefinder reading

void shiftValues(uint16_t* arr, int length) {
    for (int i = length - 1; i > 0; i--){
        arr[i] = arr[i - 1];
    }
}

void resetArray (uint16_t* arr, int length) {
    for (int i = length - 1; i > 0; i--){
        arr[i] = 0;
    }
}

// checks to see if the latest rangefinder reading makes sense and should be considered valid
bool Sub::isRangeReadingValid() {
    // use the first input values you get until you have enough to compare
    if (count < samples) {
        last_valid_rngfnd_alt = rngfnd_alts[0];
        return true;
    }

    //find differences btwn previous value
    int changeBaroAlt;
    int changeRangeAlt;

    int closeTo = 0;
    // a rapid change of > .1 m could suggest an issue
    for (int i = 1; i < samples; i++) {
        // if any of the readings are close to the new value 
        if (abs(rngfnd_alts[0] - rngfnd_alts[i]) < 10) {
            if (closeTo == 0) {
                //what was the difference between the new and most recent similar value
                changeRangeAlt = abs(rngfnd_alts[0] - rngfnd_alts[i]);
                changeBaroAlt = abs(baro_alts[0] - baro_alts[i]);
            }
            closeTo ++;
        }
    }
    // if out of range of over half of samples, then it's probably locked onto wrong target
    if (closeTo < samples / 2) {
        return false;
    }    
    //by here we know it makes some sense in relation to its  previous values

    // now check if its change is consistent with the change in depth
    if (abs(changeRangeAlt - changeBaroAlt) > 10) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "alt change disparity (%d)",
            changeRangeAlt - changeBaroAlt);
            return false;
    }
    // now we know it's consistent with the depth of the sub

    // must also be within 10 of last valid to be new valid
    last_valid_rngfnd_alt = rngfnd_alts[0];
    return true;
}

// terrfollow_init - initialise althold controller
bool Sub::terrfollow_init()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if(!control_check_barometer()) {
        return false;
    }
#endif
    //dont enter mode if rangefinder doesnt work
    if (!rangefinder_alt_ok()) {        
        // needs to output error message as well
        gcs_send_text(MAV_SEVERITY_ERROR, "Rangefinder error, cant enter terrain follow mode");
        return false;
    }

    count = 0;
    resetArray(rngfnd_alts,samples);
    resetArray(baro_alts, samples);
    last_valid_rngfnd_alt = -1;
    // dont count as near bottom until within 10 m of it
    near_bottom = rangefinder.distance_cm(0) < 1000;
    
    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);
    target_rangefinder_alt = desired_distance_from_floor;
    // initialise desired velocity
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// terrfollow_run - runs the terrfollow controller
// should be called at 100hz or more
void Sub::terrfollow_run()
{
    if (count < 15) { count++; }

    rngfnd_alts[0] = rangefinder_state.alt_cm;
    baro_alts[0] = inertial_nav.get_altitude();

    uint32_t tnow = AP_HAL::millis();

    if (!isRangeReadingValid()) {
        int avg = (rngfnd_alts[0] + rngfnd_alts[1]) / 2;
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "range reading invalid, using average of %d", avg);
        last_valid_rngfnd_alt = avg;
    }

    shiftValues(rngfnd_alts, 5);
    shiftValues(baro_alts, 5);

    if (inertial_nav.get_altitude() > 9999) {
        //dont go to 100m down. switch to alt hold if within 3m of it
        gcs_send_text(MAV_SEVERITY_ALERT, "#approaching 100m depth, switching to depth hold");
        set_mode(ALT_HOLD, MODE_REASON_BAD_DEPTH);
        return;
    }

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), 
        target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch,
                target_yaw_rate, get_smoothing_gain());
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, 
                last_pilot_heading, true, get_smoothing_gain());
        }
    }

    // Hold actual position until zero derivative is detected
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;

    if (ap.at_bottom) {
        gcs_send_text(MAV_SEVERITY_INFO, "#at bottom");
        pos_control.relax_alt_hold_controllers(); // clear velocity and position targets
        pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
    } else 
    {
        if (rangefinder_alt_ok()) {          
            float target_climb_rate = get_surface_tracking_climb_rate_terrfollow(0,
                pos_control.get_alt_target(), G_Dt, last_valid_rngfnd_alt);

            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        }
        else {
            gcs_send_text(MAV_SEVERITY_ALERT, "#rangefinder error. Switching to depth hold mode");
            set_mode(ALT_HOLD, MODE_REASON_TERRAIN_FAILSAFE);
            return;
        }
    }

    // Detects a zero derivative
    // When detected, move the altitude set point to the actual position
    // This will avoid any problem related to joystick delays
    // or smaller input signals
    if(engageStopZ && (lastVelocityZWasNegative ^ (inertial_nav.get_velocity_z() < 0) )) {
        engageStopZ = false;
        pos_control.relax_alt_hold_controllers();
    }

    pos_control.update_z_controller();

    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
