// #include "AC_CustomControl_config.h"

// // #if AP_CUSTOMCONTROL_EMPTY_ENABLED

// #include "AC_CustomControl_MTSAC.h"
// #include <GCS_MAVLink/GCS.h>
// #include <vector>

// #include "util.h"
// #include "FIFOBuffer.h"
// #include "NN_Parameters.h"

// // Initialize the class
// AC_CustomControl_MTSAC::AC_CustomControl_MTSAC(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, 
//                                            AC_AttitudeControl *&att_control, AP_MotorsMulticopter *&motors, float dt)
//     : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt), 
//       fifoBuffer(NN::N_STACK), latent_z(NN::N_LATENT, 0.0), NN_out(NN::N_ACT, 0.0)
// {
//     AP_Param::setup_object_defaults(this, AC_CustomControl_MTSAC::var_info);

//     if (NN::USE_ADAPTOR) {
//         resetAdaptorBuffer();
//     }
// }

// // Reset buffer for the adaptor
// void AC_CustomControl_MTSAC::resetAdaptorBuffer() {
//     std::vector<float> zeros_state(NN::N_OBS, 0.0);
//     for (int i = 0; i < NN::N_STACK; ++i) {
//         fifoBuffer.insert(zeros_state);
//     }
// }

// // Handle spool state and reset controller if needed
// void AC_CustomControl_MTSAC::handleSpoolState() {
//     switch (_motors->get_spool_state()) {
//         case AP_Motors::SpoolState::SHUT_DOWN:
//         case AP_Motors::SpoolState::GROUND_IDLE:
//             reset();
//             break;
//         case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//         case AP_Motors::SpoolState::SPOOLING_UP:
//         case AP_Motors::SpoolState::SPOOLING_DOWN:
//             break;
//     }
// }

// // Update NN input and handle state
// void AC_CustomControl_MTSAC::updateNNInput(const Vector3f& attitude_body, const Vector3f& gyro_latest, 
//                                           const Vector3f& airspeed_earth_ned) {
//     // Convert angles to NED
//     float rb_angle_enu_roll = attitude_body.get_euler_pitch();
//     float rb_angle_enu_pitch = attitude_body.get_euler_roll();
//     float rb_angle_enu_yaw = -attitude_body.get_euler_yaw();

//     rb_angle_enu_roll = mapAngleToRange(rb_angle_enu_roll);
//     rb_angle_enu_pitch = mapAngleToRange(rb_angle_enu_pitch);
//     rb_angle_enu_yaw = mapAngleToRange(rb_angle_enu_yaw);

//     std::vector<float> angle = {rb_angle_enu_roll, rb_angle_enu_pitch, rb_angle_enu_yaw};
//     NN::OBS[0] = angle[0] / PI;
//     NN::OBS[1] = angle[1] / PI;
//     NN::OBS[2] = angle[2] / PI;

//     Vector3f rb_ned_angvel = gyro_latest / NN::AVEL_LIM;
//     NN::OBS[6] = rb_ned_angvel[1];
//     NN::OBS[7] = rb_ned_angvel[0];
//     NN::OBS[8] = -rb_ned_angvel[2];

//     Vector3f rb_ned_vel = airspeed_earth_ned / NN::VEL_LIM;
//     NN::OBS[9] = rb_ned_vel[1];
//     NN::OBS[10] = rb_ned_vel[0];
//     NN::OBS[11] = -rb_ned_vel[2];
// }

// // Update the controller and return the output
// Vector3f AC_CustomControl_MTSAC::update(void) {
//     handleSpoolState();

//     Quaternion attitude_body, attitude_target;
//     _ahrs->get_quat_body_to_ned(attitude_body);
//     Vector3f gyro_latest = _ahrs->get_gyro_latest();
//     attitude_target = _att_control->get_attitude_target_quat();

//     float target_angle_enu_roll = attitude_target.get_euler_pitch();
//     float target_angle_enu_pitch = attitude_target.get_euler_roll();
//     float target_angle_enu_yaw = -attitude_target.get_euler_yaw();

//     updateNNInput(attitude_body, gyro_latest, _ahrs->airspeed_vector());

//     if (++adaptor_counter >= ADAPTOR_FREQ) {
//         latent_z = forward_adaptor();
//         adaptor_counter = 0;
//     }

//     if (++policy_counter >= POLICY_FREQ) {
//         if (NN::USE_ADAPTOR) {
//             NN_out = forward_adaptive_policy(NN::OBS, latent_z);
//         } else {
//             NN_out = forward_policy(NN::OBS);
//         }
//         policy_counter = 0;
//     }

//     // Prepare and return motor output
//     Vector3f motor_out;
//     motor_out.x = authority * NN_out[1];
//     motor_out.y = authority * NN_out[0];
//     motor_out.z = -authority * NN_out[2];

//     NN::OBS[12] = NN_out[0];
//     NN::OBS[13] = NN_out[1];
//     NN::OBS[14] = NN_out[2];

//     if (NN::USE_ADAPTOR) {
//         fifoBuffer.insert(NN::OBS);
//     }

//     // ###### Printing ######

//     // Log observations
//     std::string print_Str = vectorToString({NN::OBS[3], NN::OBS[4], NN::OBS[5]});
//     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "error angle: %s", print_Str.c_str());

//     // printing the output of the Network
//     // std::string NN_outStr = vectorToString(NN_out);
//     // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNout: %s", NN_outStr.c_str());

//     // printing the inference time of the Network
//     // duration<float, std::milli> ms_float = t2 - t1;
//     // float loop_frequency = 1000 / ms_float.count();
//     // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN freq: %s", std::to_string(loop_frequency).c_str());

//     // ###### Printing ######


//     return motor_out;
// }

// // Reset the controller to avoid build up or ensure bumpless transfer
// void AC_CustomControl_MTSAC::reset(void) {
//     policy_counter = 0;
//     if (NN::USE_ADAPTOR) {
//         resetAdaptorBuffer();
//         adaptor_counter = 0;
//     }
// }

// // Policy forward pass
// std::vector<float> AC_CustomControl_MTSAC::forward_policy(const std::vector<float>& state) {
//     std::vector<float> context_input = vecCat(state, NN::TASK);
//     std::vector<float> w = linear_layer(NN::WIN_B, NN::WIN_W, context_input, true);
//     w = linear_layer(NN::WOUT_B, NN::WOUT_W, w, false);
//     w = softmax(w);

//     std::vector<float> x = composition_layer(NN::LIN_W, NN::LIN_B, w, state, true);
//     x = composition_layer(NN::MEAN_W, {}, w, x, false);
//     clampToRange(x, -1, 1);

//     return x;
// }

// // Adaptive policy forward pass
// std::vector<float> AC_CustomControl_MTSAC::forward_adaptive_policy(const std::vector<float>& state, const std::vector<float>& z) {
//     std::vector<float> context_input = vecCat(state, z);
//     std::vector<float> w = linear_layer(NN::WIN_B, NN::WIN_W, context_input, true);
//     w = linear_layer(NN::WOUT_B, NN::WOUT_W, w, false);
//     w = softmax(w);

//     std::vector<float> x = composition_layer(NN::LIN_W, NN::LIN_B, w, state, true);
//     x = composition_layer(NN::MEAN_W, {}, w, x, false);
//     clampToRange(x, -1, 1);

//     return x;
// }

// // Adaptor forward pass
// std::vector<float> AC_CustomControl_MTSAC::forward_adaptor(void) {
//     std::vector<std::vector<float>> table = fifoBuffer.getTransposedTable();
//     std::vector<std::vector<float>> x_tmp1 = conv1d(table, NN::CNN_W1, NN::CNN_B1, 1, NN::N_PADD, 1);
//     std::vector<std::vector<float>> x_tmp2 = chomp1d(x_tmp1, NN::N_PADD);
//     x_tmp2 = relu2D(x_tmp2);
//     x_tmp2 = vec2DAdd(x_tmp2, table);
//     x_tmp2 = relu2D(x_tmp2);

//     if (NN::N_TCN_LAYER > 1) {
//         x_tmp1 = conv1d(x_tmp2, NN::CNN_W2, NN::CNN_B2, 1, NN::N_PADD * 2, 2);
//         x_tmp2 = chomp1d(x_tmp1, NN::N_PADD * 2);
//         x_tmp2 = relu2D(x_tmp2);
//         x_tmp2 = vec2DAdd(x_tmp2, x_tmp1);
//         x_tmp2 = relu2D(x_tmp2);
//     }

//     std::vector<float> z_tmp = getLastColumn(x_tmp2);
//     std::vector<float> z = linear_layer(NN::CNN_LB, NN::CNN_LW, z_tmp, false);
//     z = linear_layer(NN::CNN_LIN_B, NN::CNN_LIN_W, z, true);
//     z = linear_layer(NN::CNN_LOUT_B, NN::CNN_LOUT_W, z, false);

//     return z;
// }

// ////////////////////////////////////////////////////////////

// table of user settable parameters
// const AP_Param::GroupInfo AC_CustomControl_MTSAC::var_info[] = {
//     // @Param: PARAM1
//     // @DisplayName: MTSAC param1
//     // @Description: Dummy parameter for empty custom controller backend
//     // @User: Advanced
//     AP_GROUPINFO("AUTHORITY", 1, AC_CustomControl_MTSAC, authority, NN::AUTHORITY),

//     AP_GROUPEND};

// // initialize in the constructor
// AC_CustomControl_MTSAC::AC_CustomControl_MTSAC(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, AC_AttitudeControl *&att_control, AP_MotorsMulticopter *&motors, float dt) : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
// {
//     AP_Param::setup_object_defaults(this, var_info);

//     if (NN::USE_ADAPTOR==true){
//         // initialize buffer with all zeros
//         std::vector<float> zeros_state(NN::N_OBS, 0.0); 
//         for (int i = 0; i < NN::N_STACK; ++i){
//             fifoBuffer.insert(zeros_state);
//         }
//     }

// }

// // update controller
// // return roll, pitch, yaw controller output
// Vector3f AC_CustomControl_MTSAC::update(void)
// {
//     // reset controller based on spool state
//     switch (_motors->get_spool_state())
//     {
//     case AP_Motors::SpoolState::SHUT_DOWN:
//     case AP_Motors::SpoolState::GROUND_IDLE:
//         // We are still at the ground. Reset custom controller to avoid
//         // build up, ex: integrator
//         reset();
//         break;

//     case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//     case AP_Motors::SpoolState::SPOOLING_UP:
//     case AP_Motors::SpoolState::SPOOLING_DOWN:
//         // we are off the ground
//         break;
//     }

//     Quaternion attitude_body, attitude_target;
//     _ahrs->get_quat_body_to_ned(attitude_body);

//     Vector3f gyro_latest = _ahrs->get_gyro_latest();
//     attitude_target = _att_control->get_attitude_target_quat();

//     float rb_angle_enu_roll = attitude_body.get_euler_pitch();
//     float rb_angle_enu_pitch = attitude_body.get_euler_roll();
//     float rb_angle_enu_yaw = -attitude_body.get_euler_yaw();

//     rb_angle_enu_roll = mapAngleToRange(rb_angle_enu_roll);
//     rb_angle_enu_pitch = mapAngleToRange(rb_angle_enu_pitch);
//     rb_angle_enu_yaw = mapAngleToRange(rb_angle_enu_yaw);

//     float target_angle_enu_roll = attitude_target.get_euler_pitch();
//     float target_angle_enu_pitch = attitude_target.get_euler_roll();
//     float target_angle_enu_yaw = -attitude_target.get_euler_yaw();

//     float error_angle_enu_roll = target_angle_enu_roll-rb_angle_enu_roll;
//     float error_angle_enu_pitch = target_angle_enu_pitch-rb_angle_enu_pitch;
//     float error_angle_enu_yaw = target_angle_enu_yaw-rb_angle_enu_yaw;

//     error_angle_enu_roll = mapAngleToRange(error_angle_enu_roll);
//     error_angle_enu_pitch = mapAngleToRange(error_angle_enu_pitch);
//     error_angle_enu_yaw = mapAngleToRange(error_angle_enu_yaw);

//     Vector3f airspeed_earth_ned = _ahrs->airspeed_vector();
//     // Vector3f airspeed_body_ned = _ahrs->earth_to_body(airspeed_earth_ned);

//     // ###### Prepare NN input ######
//     // Note: Sensor: NED coordinate; NN input: ENU coordinate
//     // angle
//     std::vector<float> angle = {rb_angle_enu_roll, rb_angle_enu_pitch, rb_angle_enu_yaw};
//     NN::OBS[0] = angle[0]/PI;
//     NN::OBS[1] = angle[1]/PI;
//     NN::OBS[2] = angle[2]/PI;
//     // error angle
//     std::vector<float> error_angle = {error_angle_enu_roll, error_angle_enu_pitch, error_angle_enu_yaw};
//     NN::OBS[3] = error_angle[0]/PI;
//     NN::OBS[4] = error_angle[1]/PI;
//     NN::OBS[5] = error_angle[2]/PI;
//     // angular velocity
//     Vector3f rb_ned_angvel = gyro_latest/NN::AVEL_LIM;
//     NN::OBS[6] = rb_ned_angvel[1];
//     NN::OBS[7] = rb_ned_angvel[0];
//     NN::OBS[8] = -rb_ned_angvel[2];

//     Vector3f rb_ned_vel = airspeed_earth_ned/NN::VEL_LIM;
//     NN::OBS[9] = rb_ned_vel[1];
//     NN::OBS[10] = rb_ned_vel[0];
//     NN::OBS[11] = -rb_ned_vel[2];

//     // ###### Inference Starts ######
//     // auto t1 = high_resolution_clock::now();

//     adaptor_counter += 1;
//     if (adaptor_counter>=ADAPTOR_FREQ){
//         latent_z = forward_adaptor();
//         adaptor_counter = 0;
//     }

//     policy_counter += 1;
//     if (policy_counter>=POLICY_FREQ){
//         if (NN::USE_ADAPTOR==true){
//             NN_out = forward_adaptive_policy(NN::OBS, latent_z);
//         }
//         else{
//             NN_out = forward_policy(NN::OBS);
//         }
//         policy_counter = 0;
//     }

//     // auto t2 = high_resolution_clock::now();

//     // ###### Inference Ends ######


//     // include action to the observations

//     // return what arducopter main controller outputted

//     Vector3f motor_out;
//     motor_out.x = authority*NN_out[1];
//     motor_out.y = authority*NN_out[0];
//     motor_out.z = -authority*NN_out[2];

//     NN::OBS[12] = NN_out[0];
//     NN::OBS[13] = NN_out[1];
//     NN::OBS[14] = NN_out[2];

//     if (NN::USE_ADAPTOR==true){
//         fifoBuffer.insert(NN::OBS); 
//     }

//     // ###### Printing ######

//     // printing the obseravtions
//     std::string print_Str = vectorToString(error_angle);
//     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "error angle: %s", print_Str.c_str());

//     // printing the output of the Network
//     // std::string NN_outStr = vectorToString(NN_out);
//     // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNout: %s", NN_outStr.c_str());

//     // printing the inference time of the Network
//     // duration<float, std::milli> ms_float = t2 - t1;
//     // float loop_frequency = 1000 / ms_float.count();
//     // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN freq: %s", std::to_string(loop_frequency).c_str());

//     // ###### Printing ######

//     return motor_out;
// }


// // reset controller to avoid build up on the ground
// // or to provide bumpless transfer from arducopter main controller
// void AC_CustomControl_MTSAC::reset(void)
// {
//     policy_counter = 0;

//     // initialize buffer with all zeros
//     if (NN::USE_ADAPTOR==true){
//         std::vector<float> zero_state(NN::N_OBS, 0.0);
//         for (int i = 0; i < NN::N_STACK; ++i){
//             fifoBuffer.insert(zero_state);
//         }

//         adaptor_counter = 0;
//     }
// }


// std::vector<float> AC_CustomControl_MTSAC::forward_policy(std::vector<float> state)
// {
//     // policy start here
//     std::vector<float> obs = state;
//     std::vector<float> context_input = vecCat(obs, NN::TASK);

//     // context encoder
//     std::vector<float> w;
//     w = linear_layer(NN::WIN_B, NN::WIN_W, context_input, true);
//     w = linear_layer(NN::WOUT_B, NN::WOUT_W, w, false);
//     w = softmax(w);
//     assert(w.size() == NN::N_CONTEXT);

//     // composition layers
//     std::vector<float> x;
//     x = composition_layer(NN::LIN_W, NN::LIN_B, w, obs, true);

//     // output layer
//     std::vector<std::vector<float>> empty_bias;
//     x = composition_layer(NN::MEAN_W, empty_bias, w, x, false);
//     clampToRange(x, -1, 1);
//     assert(x.size() == NN::N_ACT);

//     return x;
// }

// std::vector<float> AC_CustomControl_MTSAC::forward_adaptive_policy(std::vector<float> state, std::vector<float> z)
// {
//     // policy start here
//     std::vector<float> obs = vecCat(state, z);
//     std::vector<float> context_input = vecCat(obs, NN::TASK);

//     // context encoder
//     std::vector<float> w;
//     w = linear_layer(NN::WIN_B, NN::WIN_W, context_input, true);
//     w = linear_layer(NN::WOUT_B, NN::WOUT_W, w, false);
//     w = softmax(w);
//     assert(w.size() == NN::N_CONTEXT);

//     // composition layers
//     std::vector<float> x;
//     x = composition_layer(NN::LIN_W, NN::LIN_B, w, obs, true);

//     // output layer
//     std::vector<std::vector<float>> empty_bias;
//     x = composition_layer(NN::MEAN_W, empty_bias, w, x, false);
//     clampToRange(x, -1, 1);
//     assert(x.size() == NN::N_ACT);

//     return x;
// }

// std::vector<float> AC_CustomControl_MTSAC::forward_adaptor(void)
// {
//     std::vector<std::vector<float>> table = fifoBuffer.getTransposedTable();
//     assert(table.size() == NN::N_OBS);
//     assert(table[0].size() == NN::N_STACK);

//     std::vector<std::vector<float>> x_tmp1 = conv1d(table, NN::CNN_W1, NN::CNN_B1, 1, NN::N_PADD, 1);
//     std::vector<std::vector<float>> x_tmp2 = chomp1d(x_tmp1, NN::N_PADD);
//     x_tmp2 = relu2D(x_tmp2);
//     x_tmp2 = vec2DAdd(x_tmp2, table);
//     std::vector<std::vector<float>> x = relu2D(x_tmp2);

//     if (NN::N_TCN_LAYER>1) {
//         x_tmp1 = conv1d(x, NN::CNN_W2, NN::CNN_B2, 1, NN::N_PADD*2, 2);
//         x_tmp2 = chomp1d(x_tmp1, NN::N_PADD*2);
//         x_tmp2 = relu2D(x_tmp2);
//         x = vec2DAdd(x_tmp2, x);
//         x = relu2D(x);
//     }

//     std::vector<float> z_tmp = getLastColumn(x);
//     std::vector<float> z = linear_layer(NN::CNN_LB, NN::CNN_LW, z_tmp, false);
//     z = linear_layer(NN::CNN_LIN_B, NN::CNN_LIN_W, z, true);
//     z = linear_layer(NN::CNN_LOUT_B, NN::CNN_LOUT_W, z, false);
//     assert(z.size() == NN::N_LATENT);

//     return z;
// }

// // #endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
