#pragma once
#include "AC_CustomControl_config.h"
#include "AC_CustomControl_Backend.h"
#include <vector>

class AC_CustomControl_XYZ : public AC_CustomControl_Backend {
public:
	AC_CustomControl_XYZ(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);
    Vector3f update(void) override;
    void reset(void) override;
    static const struct AP_Param::GroupInfo var_info[];

    std::vector<float> forward_adaptor(void);
    std::vector<float> forward_policy(std::vector<float>, std::vector<float>);

protected:
	AP_Float authority;
};