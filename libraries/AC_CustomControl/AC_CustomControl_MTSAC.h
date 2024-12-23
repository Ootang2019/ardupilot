// #pragma once
// #include "AC_CustomControl_config.h"
// #include "AC_CustomControl_Backend.h"
// #include <vector>

// class AC_CustomControl_MTSAC : public AC_CustomControl_Backend {
// private:
//     int policy_counter = 0;
//     int adaptor_counter = 0;
//     int POLICY_FREQ = 4;
//     int ADAPTOR_FREQ = 4;
//     FIFOBuffer fifoBuffer;
//     std::vector<float> latent_z;
//     std::vector<float> NN_out;
//     float authority;

//     // Private helper methods
//     void resetAdaptorBuffer();
//     void updateNNInput(const Vector3f& attitude_body, const Vector3f& gyro_latest, const Vector3f& airspeed_earth_ned);
//     void handleSpoolState();

// public:
//     // Constructor
//     AC_CustomControl_MTSAC(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, AC_AttitudeControl *&att_control, 
//                          AP_MotorsMulticopter *&motors, float dt);

//     // Public methods
//     Vector3f update(void) override;
//     void reset(void) override;
//     std::vector<float> forward_policy(const std::vector<float>& state);
//     std::vector<float> forward_adaptive_policy(const std::vector<float>& state, const std::vector<float>& z);
//     std::vector<float> forward_adaptor(void);

// protected:
// 	AP_Float authority;
// };