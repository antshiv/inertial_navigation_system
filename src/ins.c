#include "ins.h"

#include <math.h>
#include <string.h>

static int finite_values(const double* values, int count) {
    if (!values) {
        return 0;
    }
    for (int i = 0; i < count; ++i) {
        if (!isfinite(values[i])) {
            return 0;
        }
    }
    return 1;
}

static int valid_config(const ins_config_t* config) {
    return config && isfinite(config->nominal_dt) &&
           isfinite(config->min_dt) && isfinite(config->max_dt) &&
           isfinite(config->complementary_alpha) &&
           config->nominal_dt > 0.0 && config->min_dt > 0.0 &&
           config->max_dt >= config->min_dt &&
           config->nominal_dt >= config->min_dt &&
           config->nominal_dt <= config->max_dt &&
           config->complementary_alpha >= 0.0 &&
           config->complementary_alpha <= 1.0;
}

ins_result_t ins_init_checked(ins_t* ins, const ins_config_t* config) {
    if (!ins || !valid_config(config)) {
        return ins ? INS_INVALID_CONFIG : INS_INVALID_ARGUMENT;
    }

    ins_t initialized;
    memset(&initialized, 0, sizeof(initialized));
    initialized.config = *config;

    AttitudeEstConfig attitude_config;
    memset(&attitude_config, 0, sizeof(attitude_config));
    attitude_config.type = ESTIMATOR_COMPLEMENTARY;
    attitude_config.alpha = config->complementary_alpha;
    attitude_config.dt = config->nominal_dt;
    if (!attitude_estimator_init_checked(
            &initialized.attitude, &attitude_config)) {
        return INS_ESTIMATOR_FAILURE;
    }

    *ins = initialized;
    return INS_OK;
}

ins_result_t ins_update_checked(ins_t* ins, const ins_sensor_sample_t* sample) {
    if (!ins || !sample || !valid_config(&ins->config) ||
        !isfinite(sample->timestamp) || !finite_values(sample->gyro, 3) ||
        !finite_values(sample->accel, 3) ||
        (sample->mag_valid && !finite_values(sample->mag, 3))) {
        return INS_INVALID_ARGUMENT;
    }

    double dt = ins->config.nominal_dt;
    if (ins->has_timestamp) {
        dt = sample->timestamp - ins->last_timestamp;
        if (!isfinite(dt) || dt < ins->config.min_dt ||
            dt > ins->config.max_dt) {
            return INS_INVALID_TIMESTAMP;
        }
    }

    ins_t updated = *ins;
    const double* mag = sample->mag_valid ? sample->mag : NULL;
    if (!attitude_estimator_update_with_dt_checked(
            &updated.attitude, sample->gyro, sample->accel, mag, dt)) {
        return INS_ESTIMATOR_FAILURE;
    }
    updated.last_timestamp = sample->timestamp;
    updated.has_timestamp = 1;
    *ins = updated;
    return INS_OK;
}

ins_result_t ins_get_output_checked(const ins_t* ins, ins_output_t* output) {
    if (!ins || !output) {
        return INS_INVALID_ARGUMENT;
    }
    memset(output, 0, sizeof(*output));
    if (!valid_config(&ins->config) || !finite_values(ins->attitude.q, 4) ||
        !finite_values(ins->attitude.omega, 3)) {
        return INS_ESTIMATOR_FAILURE;
    }
    output->timestamp = ins->last_timestamp;
    memcpy(output->quaternion, ins->attitude.q, sizeof(output->quaternion));
    memcpy(output->angular_rate, ins->attitude.omega,
           sizeof(output->angular_rate));
    output->initialized = ins->attitude.initialized;
    return INS_OK;
}
