#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_t_ctrl *pid, float kp, float ki, float kd,
              float out_min, float out_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_max = integral_max;
    pid_reset(pid);
}

void pid_reset(pid_t_ctrl *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev = 0;
}

void pid_set_gains(pid_t_ctrl *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(pid_t_ctrl *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f; /* bad timestep: do nothing rather than divide by zero */
    }

    float error = setpoint - measurement;

    /* Proportional */
    float p = pid->kp * error;

    /* Derivative on measurement (negative sign: rising measurement damps output) */
    float d = 0.0f;
    if (pid->has_prev) {
        d = -pid->kd * (measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;
    pid->has_prev = 1;

    /* Integral, tentatively updated */
    float new_integral = pid->integral + pid->ki * error * dt;
    if (pid->integral_max > 0.0f) {
        new_integral = clampf(new_integral, -pid->integral_max, pid->integral_max);
    }

    float out = p + new_integral + d;
    float clamped = clampf(out, pid->out_min, pid->out_max);

    /* Anti-windup: only keep the new integral if the output isn't saturated,
     * or if the error is pulling the output back out of saturation. */
    if (out == clamped || (out > pid->out_max && error < 0.0f) ||
        (out < pid->out_min && error > 0.0f)) {
        pid->integral = new_integral;
    }

    return clamped;
}