#include "AC_CustomControl_config.h"

// #if AP_CUSTOMCONTROL_EMPTY_ENABLED

#include "AC_CustomControl_XYZ.h"
#include <GCS_MAVLink/GCS.h>
#include <vector>

#include "util.h"
#include "FIFOBuffer.h"
#include "NN_Parameters.h"

const float PI = 3.14159265358979323846;

bool adaptor_initialized=false;
int adaptor_counter=0;
int ADAPTOR_FREQ=4;
std::vector<float> latent_z(NN::N_LATENT, 0.0);
std::vector<float> NN_out(NN::N_ACT, 0.0);

FIFOBuffer fifoBuffer(NN::N_STACK);

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_XYZ::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: XYZ param1
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("AUTHORITY", 1, AC_CustomControl_XYZ, authority, NN::AUTHORITY),

    AP_GROUPEND};

// initialize in the constructor
AC_CustomControl_XYZ::AC_CustomControl_XYZ(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, AC_AttitudeControl *&att_control, AP_MotorsMulticopter *&motors, float dt) : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    // initialize buffer with all zeros
    std::vector<float> zeros_state(NN::N_OBS, 0.0); 
    for (int i = 0; i < NN::N_STACK; ++i){
        fifoBuffer.insert(zeros_state);
    }
    // latent_z = forward_adaptor();

    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_XYZ::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state())
    {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // We are still at the ground. Reset custom controller to avoid
        // build up, ex: integrator
        reset();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // we are off the ground
        break;
    }

    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    

    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    attitude_target = _att_control->get_attitude_target_quat();

    float rb_angle_enu_roll = attitude_body.get_euler_pitch();
    float rb_angle_enu_pitch = attitude_body.get_euler_roll();
    float rb_angle_enu_yaw = -attitude_body.get_euler_yaw();

    rb_angle_enu_roll = mapAngleToRange(rb_angle_enu_roll);
    rb_angle_enu_pitch = mapAngleToRange(rb_angle_enu_pitch);
    rb_angle_enu_yaw = mapAngleToRange(rb_angle_enu_yaw);

    float target_angle_enu_roll = attitude_target.get_euler_pitch();
    float target_angle_enu_pitch = attitude_target.get_euler_roll();
    float target_angle_enu_yaw = -attitude_target.get_euler_yaw();

    float error_angle_enu_roll = target_angle_enu_roll-rb_angle_enu_roll;
    float error_angle_enu_pitch = target_angle_enu_pitch-rb_angle_enu_pitch;
    float error_angle_enu_yaw = target_angle_enu_yaw-rb_angle_enu_yaw;

    error_angle_enu_roll = mapAngleToRange(error_angle_enu_roll);
    error_angle_enu_pitch = mapAngleToRange(error_angle_enu_pitch);
    error_angle_enu_yaw = mapAngleToRange(error_angle_enu_yaw);

    // Vector3f airspeed_earth_ned = _ahrs->airspeed_vector();
    // Vector3f airspeed_body_ned = _ahrs->earth_to_body(airspeed_earth_ned);

    // ###### Prepare NN input ######
    // Note: Sensor: NED coordinate; NN input: ENU coordinate
    // angle
    std::vector<float> angle = {rb_angle_enu_roll, rb_angle_enu_pitch, rb_angle_enu_yaw};
    NN::OBS[0] = angle[0]/PI;
    NN::OBS[1] = angle[1]/PI;
    NN::OBS[2] = angle[2]/PI;
    // error angle
    std::vector<float> error_angle = {error_angle_enu_roll, error_angle_enu_pitch, error_angle_enu_yaw};
    NN::OBS[3] = error_angle[0]/PI;
    NN::OBS[4] = error_angle[1]/PI;
    NN::OBS[5] = error_angle[2]/PI;
    // angular velocity
    Vector3f rb_ned_angvel = gyro_latest/NN::AVEL_LIM;
    NN::OBS[6] = rb_ned_angvel[1];
    NN::OBS[7] = rb_ned_angvel[0];
    NN::OBS[8] = -rb_ned_angvel[2];

    // ###### Inference Starts ######
    // auto t1 = high_resolution_clock::now();

    adaptor_counter += 1;
    if (adaptor_counter>=ADAPTOR_FREQ){
        latent_z = forward_adaptor();
        adaptor_counter = 0;
        adaptor_initialized = true;
        NN_out = forward_policy(NN::OBS, latent_z);
    }
    // std::vector<float> NN_out = forward_policy(NN::OBS, latent_z);

    // auto t2 = high_resolution_clock::now();

    // ###### Inference Ends ######


    // include action to the observations

    // return what arducopter main controller outputted

    Vector3f motor_out;
    if (adaptor_initialized==true){
        motor_out.x = authority*NN_out[1];
        motor_out.y = authority*NN_out[0];
        motor_out.z = -authority*NN_out[2];

        NN::OBS[9] = NN_out[0];
        NN::OBS[10] = NN_out[1];
        NN::OBS[11] = NN_out[2];
    }
    else{
        motor_out.x = 0;
        motor_out.y = 0;
        motor_out.z = 0;

        NN::OBS[9] = 0;
        NN::OBS[10] = 0;
        NN::OBS[11] = 0;
    }
    fifoBuffer.insert(NN::OBS); 

    motor_out.x = 0;
    motor_out.y = 0;
    motor_out.z = 0;
    NN::OBS[9] = 0;
    NN::OBS[10] = 0;
    NN::OBS[11] = 0;

    // ###### Printing ######

    // printing the obseravtions
    // std::string print_Str = vectorToString(error_angle);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "error angle: %s", print_Str.c_str());

    // printing the output of the Network
    // std::string NN_outStr = vectorToString(NN_out);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNout: %s", NN_outStr.c_str());

    // printing the inference time of the Network
    // duration<float, std::milli> ms_float = t2 - t1;
    // float loop_frequency = 1000 / ms_float.count();
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN freq: %s", std::to_string(loop_frequency).c_str());

    // ###### Printing ######

    return motor_out;
}

std::vector<float> AC_CustomControl_XYZ::forward_adaptor(void)
{
    std::vector<std::vector<float>> table = fifoBuffer.getTransposedTable();
    assert(table.size() == NN::N_OBS);
    assert(table[0].size() == NN::N_STACK);

    std::vector<std::vector<float>> x_tmp1 = conv1d(table, NN::CNN_W1, NN::CNN_B1, 1, NN::N_PADD, 1);
    std::vector<std::vector<float>> x_tmp2 = chomp1d(x_tmp1, NN::N_PADD);
    x_tmp2 = relu2D(x_tmp2);
    x_tmp2 = vec2DAdd(x_tmp2, table);
    std::vector<std::vector<float>> x = relu2D(x_tmp2);

    if (NN::N_TCN_LAYER>1) {
        x_tmp1 = conv1d(x, NN::CNN_W2, NN::CNN_B2, 1, NN::N_PADD*2, 2);
        x_tmp2 = chomp1d(x_tmp1, NN::N_PADD*2);
        x_tmp2 = relu2D(x_tmp2);
        x = vec2DAdd(x_tmp2, x);
        x = relu2D(x);
    }

    std::vector<float> z_tmp = getLastColumn(x);
    std::vector<float> z = linear_layer(NN::CNN_LB, NN::CNN_LW, z_tmp, false);
    z = linear_layer(NN::CNN_LIN_B, NN::CNN_LIN_W, z, true);
    z = linear_layer(NN::CNN_LOUT_B, NN::CNN_LOUT_W, z, false);
    assert(z.size() == NN::N_LATENT);

    return z;
}

std::vector<float> AC_CustomControl_XYZ::forward_policy(std::vector<float> state, std::vector<float> z)
{
    // policy start here
    std::vector<float> obs = vecCat(state, z);
    std::vector<float> context_input = vecCat(obs, NN::TASK);

    // context encoder
    std::vector<float> w;
    w = linear_layer(NN::WIN_B, NN::WIN_W, context_input, true);
    w = linear_layer(NN::WOUT_B, NN::WOUT_W, w, false);
    w = softmax(w);
    assert(w.size() == NN::N_CONTEXT);

    // composition layers
    std::vector<float> x;
    x = composition_layer(NN::LIN_W, NN::LIN_B, w, obs, true);
    // x = composition_layer(NN::L0_W, NN::L0_B, w, x, true);

    // output layer
    std::vector<std::vector<float>> empty_bias;
    x = composition_layer(NN::MEAN_W, empty_bias, w, x, false);
    clampToRange(x, -1, 1);
    assert(x.size() == NN::N_ACT);

    return x;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_XYZ::reset(void)
{
    // initialize buffer with all zeros
    std::vector<float> zero_state(NN::N_OBS, 0.0);
    for (int i = 0; i < NN::N_STACK; ++i){
        fifoBuffer.insert(zero_state);
    }

    adaptor_counter = 0;
    adaptor_initialized = false;
}

// #endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
