#include "mode.h"
#include "Plane.h"

/*
  FBWA mode parameters
 */
const AP_Param::GroupInfo ModeFBWA::var_info[] = {
    // @Param: ROLL_LIM
    // @DisplayName: BSA: FBWA roll limit
    // @Description: Maximum roll angle in FBWA mode. Set to 0 to use the global ROLL_LIMIT_DEG parameter instead.
    // @Range: 0 90
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ROLL_LIM", 1, ModeFBWA, roll_limit, 0),

    // @Param: PTCH_MAX
    // @DisplayName: BSA: FBWA maximum pitch up
    // @Description: Maximum pitch-up angle in FBWA mode. Set to 0 to use the global PTCH_LIM_MAX_DEG parameter instead.
    // @Range: 0 90
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PTCH_MAX", 2, ModeFBWA, pitch_max, 0),

    // @Param: PTCH_MIN
    // @DisplayName: BSA: FBWA maximum pitch down
    // @Description: Maximum pitch-down angle in FBWA mode (enter as a negative value). Set to 0 to use the global PTCH_LIM_MIN_DEG parameter instead.
    // @Range: -90 0
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PTCH_MIN", 3, ModeFBWA, pitch_min, 0),

    AP_GROUPEND
};

ModeFBWA::ModeFBWA() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void ModeFBWA::update()
{
    // Raise roll_limit_cd to the FBWA ceiling before update_load_factor() so that
    // apply_load_factor_roll_limits() operates relative to the FBWA limit, not ROLL_LIMIT_DEG.
    if (roll_limit.get() > 0.0f) {
        const int32_t fbwa_roll_limit_cd = int32_t(roll_limit.get() * fabsf(plane.ahrs.cos_pitch()) * 100);
        plane.roll_limit_cd = MAX(plane.roll_limit_cd, fbwa_roll_limit_cd);
    }

    const float fbwa_pitch_max = (pitch_max > 0) ? pitch_max.get() : plane.aparm.pitch_limit_max.get();
    // Mirror the cos_roll scaling Plane.cpp applies to the global pitch_limit_min each loop.
    const float fbwa_pitch_min = (pitch_min < 0) ? pitch_min.get() * fabsf(plane.ahrs.cos_roll()) : plane.pitch_limit_min;

    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * fbwa_pitch_max * 100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * fbwa_pitch_min * 100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, fbwa_pitch_min * 100, fbwa_pitch_max * 100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

void ModeFBWA::run()
{
    // Run base class function and then output throttle
    Mode::run();

    output_pilot_throttle();
}
