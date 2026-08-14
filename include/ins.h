#ifndef ANTSHIV_INS_H
#define ANTSHIV_INS_H

#include "estimators/attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    INS_OK = 0,
    INS_INVALID_ARGUMENT = 1,
    INS_INVALID_CONFIG = 2,
    INS_INVALID_TIMESTAMP = 3,
    INS_ESTIMATOR_FAILURE = 4
} ins_result_t;

typedef struct {
    double nominal_dt;
    double min_dt;
    double max_dt;
    double complementary_alpha;
} ins_config_t;

typedef struct {
    double timestamp;
    double gyro[3];
    double accel[3];
    double mag[3];
    int mag_valid;
} ins_sensor_sample_t;

typedef struct {
    double timestamp;
    double quaternion[4];
    double angular_rate[3];
    int initialized;
} ins_output_t;

typedef struct {
    ins_config_t config;
    AttitudeEstimator attitude;
    double last_timestamp;
    int has_timestamp;
} ins_t;

ins_result_t ins_init_checked(ins_t* ins, const ins_config_t* config);
ins_result_t ins_update_checked(ins_t* ins, const ins_sensor_sample_t* sample);
ins_result_t ins_get_output_checked(const ins_t* ins, ins_output_t* output);

#ifdef __cplusplus
}
#endif

#endif
