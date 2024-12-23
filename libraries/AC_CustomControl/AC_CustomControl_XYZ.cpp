#include "AC_CustomControl_config.h"
// #if AP_CUSTOMCONTROL_EMPTY_ENABLED

#include "AC_CustomControl_XYZ.h"
#include <GCS_MAVLink/GCS.h>
#include <vector>
#include <cmath>  // for fmod, etc.

#include "util.h"           // includes your updated utility + GraphNN declarations
#include "NN_Parameters.h"  // includes your trained model parameters

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_XYZ::var_info[] = {
    AP_GROUPINFO("AUTHORITY", 1, AC_CustomControl_XYZ, authority, NN::AUTHORITY),
    AP_GROUPEND
};

AC_CustomControl_XYZ::AC_CustomControl_XYZ(AC_CustomControl &frontend, 
                                           AP_AHRS_View *&ahrs, 
                                           AC_AttitudeControl *&att_control, 
                                           AP_MotorsMulticopter *&motors, 
                                           float dt)
    : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, AC_CustomControl_XYZ::var_info);
}

void AC_CustomControl_XYZ::handleSpoolState() {
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            reset();
            break;
        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            break;
    }
}

// Update NN input and handle state
void AC_CustomControl_XYZ::updateNNInput(const Vector3f& attitude_body,
                                         const Vector3f& gyro_latest, 
                                         const Vector3f& airspeed_earth_ned) 
{
    float rb_angle_enu_roll  = attitude_body.get_euler_pitch();
    float rb_angle_enu_pitch = attitude_body.get_euler_roll();
    float rb_angle_enu_yaw   = -attitude_body.get_euler_yaw();

    rb_angle_enu_roll  = mapAngleToRange(rb_angle_enu_roll);
    rb_angle_enu_pitch = mapAngleToRange(rb_angle_enu_pitch);
    rb_angle_enu_yaw   = mapAngleToRange(rb_angle_enu_yaw);

    // Just store these in NN::OBS for now, or you might store them in a local vector
    NN::OBS[0] = rb_angle_enu_roll  / PI;
    NN::OBS[1] = rb_angle_enu_pitch / PI;
    NN::OBS[2] = rb_angle_enu_yaw   / PI;

    Vector3f rb_ned_angvel = gyro_latest / NN::AVEL_LIM;
    NN::OBS[3] = rb_ned_angvel[0];
    NN::OBS[4] = rb_ned_angvel[1];
    NN::OBS[5] = rb_ned_angvel[2];

    Vector3f rb_ned_vel = airspeed_earth_ned / NN::VEL_LIM;
    NN::OBS[6]  = rb_ned_vel[0];
    NN::OBS[7]  = rb_ned_vel[1];
    NN::OBS[8]  = rb_ned_vel[2];
}

// Update the controller and return the output
Vector3f AC_CustomControl_XYZ::update(void) {
    handleSpoolState();

    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    attitude_target = _att_control->get_attitude_target_quat();

    // Just as an example, we can read out target euler if needed
    float target_angle_enu_roll  = attitude_target.get_euler_pitch();
    float target_angle_enu_pitch = attitude_target.get_euler_roll();
    float target_angle_enu_yaw   = -attitude_target.get_euler_yaw();

    updateNNInput(attitude_body, gyro_latest, _ahrs->airspeed_vector());

    // forward pass
    if (++policy_counter >= POLICY_FREQ) {
        NN_out = forward_policy(NN::OBS); 
        policy_counter = 0;
    }

    // motor outputs
    Vector3f motor_out;
    motor_out.x = authority * NN_out[1];
    motor_out.y = authority * NN_out[0];
    motor_out.z = -authority * NN_out[2];

    // Debug info
    std::string obsStr = vectorToString({NN::OBS[0], NN::OBS[1], NN::OBS[2]});
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "angles (obs0..2): %s", obsStr.c_str());

    return motor_out;
}

void AC_CustomControl_XYZ::reset(void) {
    policy_counter = 0;
}

// -----------------------------------------------------------------------------
// GCN-based policy with parallelFC for l_out and mean_linear
// -----------------------------------------------------------------------------

// A small helper to embed a list of scalars (x[]) with shape [X] into shape [X, hidden_dim] flattened
static std::vector<float> embed_scalars_1batch(
    const std::vector<float> &xs,             // [X]
    const std::vector<float> &W_2d,           // e.g. ANG_EMB_W => shape [N_hidden, 1] flattened row-major
    const std::vector<float> &b_1d,           // e.g. ANG_EMB_B => shape [N_hidden]
    int hidden_dim)
{
    // Output => [X, hidden_dim], flatten => length = X*hidden_dim
    std::vector<float> out(xs.size() * hidden_dim, 0.0f);
    for (size_t i = 0; i < xs.size(); i++) {
        float val = xs[i];
        for (int h = 0; h < hidden_dim; h++) {
            // W_2d[h*1 + 0], since in_dim=1
            float w = W_2d[h]; 
            out[i*hidden_dim + h] = w * val + b_1d[h];
        }
    }
    return out;
}

// This is your GCN forward pass
std::vector<float> AC_CustomControl_XYZ::forward_policy(const std::vector<float>& state)
{
    //----------------------------------------------------------
    // 1) Slice out angles, angvel, vel, etc. from "state" 
    //    and embed them
    //----------------------------------------------------------
    // Suppose "state" has length >= 9 => 
    //    [0..2] => angles, [3..5] => angvel, [6..8] => vel
    // For "goal" or "task", you can adapt if needed
    const int hidden_dim = NN::ANG_EMB_B.size(); // e.g. 32

    // angles => shape [3], embed => [3, hidden_dim]
    std::vector<float> rb_ang    ( state.begin()+0, state.begin()+3 );
    std::vector<float> embed_ang = embed_scalars_1batch(rb_ang, 
                                                        NN::ANG_EMB_W, 
                                                        NN::ANG_EMB_B, 
                                                        hidden_dim);

    // angvel => shape [3], embed => [3, hidden_dim]
    std::vector<float> rb_angvel ( state.begin()+3, state.begin()+6 );
    std::vector<float> embed_angvel = embed_scalars_1batch(rb_angvel,
                                                           NN::ANGVEL_EMB_W,
                                                           NN::ANGVEL_EMB_B,
                                                           hidden_dim);

    // vel => shape [3], embed => [3, hidden_dim]
    std::vector<float> rb_vel ( state.begin()+6, state.begin()+9 );
    std::vector<float> embed_vel = embed_scalars_1batch(rb_vel,
                                                        NN::VEL_EMB_W,
                                                        NN::VEL_EMB_B,
                                                        hidden_dim);

    // For "goal" or "task", if you have them in state or separate, do similarly. 
    // We'll skip "goal" for brevity, but you can replicate with e.g. ANG_EMB_W 
    // or a separate embed. If you do have a "task" in NN::TASK, also embed that:
    std::vector<float> embed_task;
    {
        // Suppose "NN::TASK" is shape [T], embed with TASK_EMB_W => shape [hidden_dim, 1]
        embed_task = embed_scalars_1batch(
            NN::TASK, // e.g. [1..some dim]
            NN::TASK
