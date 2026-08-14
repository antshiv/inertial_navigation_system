#include <assert.h>
#include <math.h>
#include <string.h>

#include "ins.h"

int main(void) {
    const ins_config_t config = {
        .nominal_dt = 0.01,
        .min_dt = 0.001,
        .max_dt = 0.02,
        .complementary_alpha = 0.98,
    };
    ins_t ins;
    assert(ins_init_checked(&ins, &config) == INS_OK);

    ins_sensor_sample_t sample = {
        .timestamp = 1.0,
        .gyro = {0.0, 0.0, 0.0},
        .accel = {0.0, 0.0, 1.0},
        .mag = {1.0, 0.0, 0.0},
        .mag_valid = 1,
    };
    assert(ins_update_checked(&ins, &sample) == INS_OK);

    ins_output_t output;
    assert(ins_get_output_checked(&ins, &output) == INS_OK);
    assert(output.initialized == 1);
    assert(fabs(output.quaternion[0] - 1.0) < 1e-12);

    const ins_t before = ins;
    sample.timestamp = 1.1;
    assert(ins_update_checked(&ins, &sample) == INS_INVALID_TIMESTAMP);
    assert(memcmp(&ins, &before, sizeof(ins)) == 0);

    sample.timestamp = 1.01;
    sample.gyro[0] = NAN;
    assert(ins_update_checked(&ins, &sample) == INS_INVALID_ARGUMENT);
    assert(memcmp(&ins, &before, sizeof(ins)) == 0);
    return 0;
}
