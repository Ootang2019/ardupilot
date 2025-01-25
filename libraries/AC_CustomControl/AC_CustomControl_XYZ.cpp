#include "AC_CustomControl_config.h"

// #if AP_CUSTOMCONTROL_XYZ_ENABLED

// #include <sstream>
// #include <iostream>

#include "AC_CustomControl_XYZ.h"
#include <GCS_MAVLink/GCS.h>
#include <vector>
#include <cmath>  // for fmod, etc.

#include "util.h"           // includes your updated utility + GraphNN declarations
#include "NN_Parameters.h"  // includes your trained model parameters

#ifndef PI
#define PI 3.14159265358979323846f
#endif

int policy_counter = 0;
int POLICY_FREQ = 4;
std::vector<float> NN_out={0.0,0.0,0.0};

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
void AC_CustomControl_XYZ::updateNNInput(const Quaternion& attitude_body,
                                         const Quaternion& attitude_target,
                                         const Vector3f& gyro_latest, 
                                         const Vector3f& airspeed_earth_ned) 
{

    float rb_angle_enu_roll  = attitude_body.get_euler_pitch();
    float rb_angle_enu_pitch = attitude_body.get_euler_roll();
    float rb_angle_enu_yaw   = attitude_body.get_euler_yaw();

    rb_angle_enu_roll  = mapAngleToRange(rb_angle_enu_roll);
    rb_angle_enu_pitch = mapAngleToRange(rb_angle_enu_pitch);
    rb_angle_enu_yaw   = mapAngleToRange(rb_angle_enu_yaw);

    float target_angle_enu_roll  = attitude_target.get_euler_pitch();
    float target_angle_enu_pitch = attitude_target.get_euler_roll();
    float target_angle_enu_yaw   = attitude_target.get_euler_yaw();

    float error_angle_enu_roll  = mapAngleToRange(target_angle_enu_roll-rb_angle_enu_roll);
    float error_angle_enu_pitch = mapAngleToRange(target_angle_enu_pitch-rb_angle_enu_pitch);
    float error_angle_enu_yaw   = mapAngleToRange(target_angle_enu_yaw-rb_angle_enu_yaw);


    // Just store these in NN::OBS for now, or you might store them in a local vector
    NN::OBS[0] = rb_angle_enu_roll  / PI;
    NN::OBS[1] = rb_angle_enu_pitch / PI;
    NN::OBS[2] = rb_angle_enu_yaw   / PI;

    Vector3f rb_ned_angvel = gyro_latest / NN::AVEL_LIM;
    NN::OBS[3] = rb_ned_angvel[1];
    NN::OBS[4] = rb_ned_angvel[0];
    NN::OBS[5] = -rb_ned_angvel[2];

    NN::OBS[9]  = error_angle_enu_roll / PI;
    NN::OBS[10]  = error_angle_enu_pitch / PI;
    NN::OBS[11]  = error_angle_enu_yaw / PI;

}

// Update the controller and return the output
Vector3f AC_CustomControl_XYZ::update(void) {
    handleSpoolState();

    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    attitude_target = _att_control->get_attitude_target_quat();

    updateNNInput(attitude_body, attitude_target, gyro_latest, _ahrs->airspeed_vector());

    // forward pass
    policy_counter += 1;
    if (policy_counter >= POLICY_FREQ) {
        policy_counter = 0;  // Reset the counter after performing the update
        NN_out = forward_policy(NN::OBS);  
    } 
    // std::string obsStr = vectorToString(NN_out);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "nnout: %s", obsStr.c_str());

    NN::OBS[6] = NN_out[0];
    NN::OBS[7] = NN_out[1];
    NN::OBS[8] = NN_out[2];
    // NN::OBS[8] = 0;

    // motor outputs
    Vector3f motor_out;
    motor_out.x = authority * NN_out[1];
    motor_out.y = authority * NN_out[0];
    motor_out.z = -authority * NN_out[2];
    // motor_out.z = 0;


    // Debug info
    // std::string obsStr = vectorToString({NN::OBS[0], NN::OBS[1], NN::OBS[2]});
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "angles (obs0..2): %s", obsStr.c_str());

    return motor_out;
}

void AC_CustomControl_XYZ::reset(void) {
}

// -----------------------------------------------------------------------------
// GCN-based policy with parallelFC for l_out and mean_linear
// -----------------------------------------------------------------------------

// A small helper to embed a list of scalars (x[]) with shape [X] into shape [X, hidden_dim] flattened
// Embed scalars with flattened weights and 1D bias
static std::vector<float> embed_scalars_1batch(
    const std::vector<float>& xs,
    const std::vector<float>& W_1d,  // Flattened weights
    const std::vector<float>& b_1d, // 1D bias
    int hidden_dim
) {
    std::vector<float> out(xs.size() * hidden_dim, 0.0f);

    for (size_t i = 0; i < xs.size(); i++) {
        float val = xs[i];
        for (int h = 0; h < hidden_dim; h++) {
            float w = W_1d[h];  // Access directly from flattened weights
            out[i * hidden_dim + h] = w * val + b_1d[h];
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
    // Suppose "state" has length >= 12 => 
    //    [0..2] => angles, [3..5] => angvel, [6..8] => vel, [9..11] => target_angles
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

    // "action_init" => shape [action_dim, hidden_dim], flatten => length = action_dim*hidden_dim
    // e.g. if N_act=3, shape => [3, hidden_dim]
    std::vector<float> prev_act ( state.begin()+6, state.begin()+9);
    std::vector<float> embed_act = embed_scalars_1batch(prev_act,
                                                        NN::ACT_EMB_W,
                                                        NN::ACT_EMB_B,
                                                        hidden_dim);
    // std::vector<float> action_init_flat = NN::ACTION_INIT;

    // goal => shape [3], embed => [3, hidden_dim]
    std::vector<float> goal_ang ( state.begin()+9, state.begin()+12 );
    std::vector<float> embed_goal_ang = embed_scalars_1batch(goal_ang,
                                                             NN::ANG_EMB_W,
                                                             NN::ANG_EMB_B,
                                                             hidden_dim);

    // task => shape [T], embed => [T, hidden_dim]
    std::vector<float> embed_task = embed_scalars_1batch(NN::TASK,
                                                         NN::TASK_EMB_W,
                                                         NN::TASK_EMB_B,
                                                         hidden_dim);

    //----------------------------------------------------------
    // 2) Concatenate all node embeddings: 
    //    angles(3) + angvel(3) + task(T) + prev_act(A)
    //----------------------------------------------------------
    // embed_ang.size()       => 3*hidden_dim
    // embed_angvel.size()    => 3*hidden_dim
    // embed_task.size()      => T*hidden_dim
    std::vector<float> H_concat;
    H_concat.insert(H_concat.end(), embed_ang.begin(),    embed_ang.end());
    H_concat.insert(H_concat.end(), embed_angvel.begin(), embed_angvel.end());
    H_concat.insert(H_concat.end(), embed_goal_ang.begin(),    embed_goal_ang.end());
    H_concat.insert(H_concat.end(), embed_task.begin(),   embed_task.end());
    H_concat.insert(H_concat.end(), embed_act.begin(),    embed_act.end());

    int num_obs_nodes    = NN::N_STATE + NN::N_GOAL + NN::N_TASK; // e.g. 16, S(6)+G(3)+T(7)
    int num_action_nodes = NN::N_ACT;                  // e.g. 3
    int total_nodes      = num_obs_nodes + num_action_nodes; // e.g. 19

    //----------------------------------------------------------
    // 3) GCN layer
    //----------------------------------------------------------
    
    // gcn0 => out => shape [total_nodes, hidden_dim], flattened
    // we have: W => GCN0_W [hidden_dim * hidden_dim], b => GCN0_B [hidden_dim], optional LN => GCN0_LN_W, GCN0_LN_B
    std::vector<float> H_gcn0 = GraphNN::gcn_1batch(
        H_concat, 
        total_nodes,
        hidden_dim,
        NN::A, 
        NN::GCN0_W, 
        NN::GCN0_B, 
        hidden_dim,
        NN::USE_LAYERNORM,
        NN::GCN0_LN_W,
        NN::GCN0_LN_B
    );

    //----------------------------------------------------------
    // 4) Slice out action nodes => [B=1, A, hidden_dim]
    //----------------------------------------------------------
    // Those nodes are [num_obs_nodes..num_obs_nodes+num_action_nodes)
    // We'll flatten them => length = A*hidden_dim
    std::vector<float> H_actions(NN::N_ACT * hidden_dim);
    for (int a = 0; a < NN::N_ACT; a++) {
        for (int h = 0; h < hidden_dim; h++) {
            H_actions[a*hidden_dim + h] = H_gcn0[(num_obs_nodes + a)*hidden_dim + h];
        }
    }

    //----------------------------------------------------------
    // 5) parallelFC => l_out => shape [A, hidden_dim]
    //    flatten => length = A*hidden_dim
    //----------------------------------------------------------
    // l_out.weight => LOUT_W => shape [A, hidden_dim, hidden_dim]
    // l_out.bias   => LOUT_B => shape [A, hidden_dim]
    // We'll use parallel_fc_1batch with relu=true
    std::vector<float> x_lout = GraphNN::parallel_fc_1batch(
        H_actions,              // [A*hidden_dim]
        NN::N_ACT,             // n_parallels
        hidden_dim,             // in_dim
        hidden_dim,             // out_dim
        NN::LOUT_W,             // shape [A, hidden_dim, hidden_dim] flattened
        NN::LOUT_B,             // shape [A, hidden_dim]
        true                    // ReLU
    );

    //----------------------------------------------------------
    // 6) parallelFC => mean_linear => shape [A,1] => flatten => [A]
    //----------------------------------------------------------
    // mean_w => MEAN_W => shape [A, 1, hidden_dim]
    // mean_b => MEAN_B => shape [A, 1]
    // so out_dim=1, relu=false
    std::vector<float> bias(NN::N_ACT, 0.0);
    std::vector<float> mean_out = GraphNN::parallel_fc_1batch(
        x_lout,                 // [A*hidden_dim]
        NN::N_ACT,             // n_parallels
        hidden_dim,             // in_dim
        1,                      // out_dim
        NN::MEAN_W,             // shape [A,1,hidden_dim]
        bias,                   // shape [A,1]
        false                   // no ReLU
    );  

    // mean_out => length = A*1 => [A]

    //----------------------------------------------------------
    // 7) clamp to [-1, 1]
    //----------------------------------------------------------
    clampToRange(mean_out, -1.0f, 1.0f);

    return mean_out;  // shape => [A]
}

// #endif  // AP_CUSTOMCONTROL_XYZ_ENABLED
