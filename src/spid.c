#include "spid.h"

#include <stdio.h>

#define MIN_LOOP_INTERVAL_S (0.00001)

spid_err_t spid_init(spid_t* ctrl, float p, float i, float d, float min, float max, float interval_s) {
    if(ctrl == NULL) return SPID_SUCCESS;
    if(p < 0 || i < 0 || d < 0 || interval_s < 0) {
        return SPID_INVALID_PARAMETER;
    }
    if(max < min) {
        return SPID_INVALID_PARAMETER;
    }
    if(interval_s < MIN_LOOP_INTERVAL_S){
        return SPID_INVALID_PARAMETER;
    }

    ctrl->k_p = p;
    ctrl->k_i = i;
    ctrl->k_d = d;

    ctrl->output_min = min;
    ctrl->output_max = max;
    ctrl->loop_interval_s = interval_s;

    ctrl->_err_prev = 0;
    ctrl->_integral_sum = 0;

    return SPID_SUCCESS;
}

float spid_process(spid_t* ctrl, float target, float measured) {
    if(ctrl == NULL) return 0;

    // error calculations
    float error = target - measured;
    ctrl->_integral_sum += error;

    // output calculation
    float p_out = ctrl->k_p * error;
    float i_out = ctrl->k_i * ctrl->_integral_sum;
    float d_out = ctrl->k_d * ((error - ctrl->_err_prev) / ctrl->loop_interval_s);

    // set err_prev for next loop
    ctrl->_err_prev = error;

    // add up terms and saturate result if required
    float out = p_out + i_out + d_out;
    if(out < ctrl->output_min) out = ctrl->output_min;
    if(out > ctrl->output_max) out = ctrl->output_max;

    return out;
}