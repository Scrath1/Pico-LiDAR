#ifndef CPID_H
#define CPID_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float k_p;              // proportional gain constant
    float k_i;              // integral gain constant
    float k_d;              // derivative gain constant
    float output_min;       // minimum output value
    float output_max;       // maximum output value
    float loop_interval_s;  // time interval with which the loop runs in seconds

    float _err_prev;      // previous proportional error for calculation of derivative
    float _integral_sum;  // sum of proportional errors for calculation of integral
} spid_t;

typedef enum {
    SPID_SUCCESS = 0,             // Success
    SPID_INVALID_PARAMETER = -1,  // Invalid parameter was passed
    SPID_NULLPTR = -2             // variable was a nullptr
} spid_err_t;

/**
 * @brief Initialize spid_t struct. The given constants must be bigger than 0
 * @param ctrl [INOUT] Controller struct to initialize
 * @param p [IN] Proportional gain constant
 * @param i [IN] Integral gain constant
 * @param d [IN] Derivative gain constant
 * @param min [IN] Minimum output value
 * @param max [IN] Maximum output value
 * @param interval_s [IN] Interval in seconds between each execution of spid_process
 * @return SPID_SUCCESS
 * @return SPID_INVALID_PARAMETER if p, i or d are negative or min is
 *  larger than max
 * @return SPID_NULLPTR if ctrl is NULL
 */
spid_err_t spid_init(spid_t* ctrl, float p, float i, float d, float min, float max, float interval_s);

/**
 * @brief
 * @param ctrl [INOUT] PID controller struct
 * @param target [IN] Target value, also known as setpoint (SP)
 * @param measured [IN] Actual value, also known as process variable (PV)
 * @return PID output value with saturation applied if required
 */
float spid_process(spid_t* ctrl, float target, float measured);

/**
 * @brief Updates the k_p gain constant
 * @return SPID_SUCCESS
 * @return SPID_NULLPTR if ctrl is NULL
 */
spid_err_t spid_set_kp(spid_t* ctrl, float p);

/**
 * @brief Updates the k_i gain constant
 * @return SPID_SUCCESS
 * @return SPID_NULLPTR if ctrl is NULL
 */
spid_err_t spid_set_ki(spid_t* ctrl, float i);

/**
 * @brief Updates the k_d gain constant
 * @return SPID_SUCCESS
 * @return SPID_NULLPTR if ctrl is NULL
 */
spid_err_t spid_set_kd(spid_t* ctrl, float d);
#ifdef __cplusplus
}
#endif
#endif  // CPID_H