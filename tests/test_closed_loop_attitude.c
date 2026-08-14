#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "attitude/quaternion.h"
#include "drone/physics_model.h"
#include "ins.h"
#include "mixer.h"
#include "pid.h"

static void diagonal(double matrix[3][3], double x, double y, double z) {
    memset(matrix, 0, 9 * sizeof(double));
    matrix[0][0] = x;
    matrix[1][1] = y;
    matrix[2][2] = z;
}

static void configure_rotor(dm_rotor_config_t* dm,
                            cs_rotor_config_t* cs,
                            double x, double y, double direction) {
    memset(dm, 0, sizeof(*dm));
    memset(cs, 0, sizeof(*cs));
    dm->position_body[0] = cs->position[0] = x;
    dm->position_body[1] = cs->position[1] = y;
    dm->axis_body[2] = cs->axis[2] = -1.0;
    dm->direction = cs->direction = direction;
    dm->thrust_coeff = cs->thrust_coeff = 1.0e-5;
    dm->torque_coeff = cs->torque_coeff = 2.0e-7;
}

static void body_vector(const double q_body_to_ned[4],
                        const double vector_ned[3], double vector_body[3]) {
    double inverse[4];
    assert(quaternion_inverse(q_body_to_ned, inverse));
    quaternion_rotate_vector(inverse, vector_ned, vector_body);
}

static double attitude_angle(const double q[4]) {
    double scalar = fabs(q[0]);
    if (scalar > 1.0) {
        scalar = 1.0;
    }
    return 2.0 * acos(scalar);
}

int main(void) {
    const double dt = 0.002;
    const double mass = 1.5;
    const double arm = 0.2;

    dm_vehicle_config_t vehicle;
    memset(&vehicle, 0, sizeof(vehicle));
    vehicle.rotor_count = 4;
    vehicle.mass = mass;
    vehicle.gravity = 9.81;
    diagonal(vehicle.inertia, 0.02, 0.025, 0.04);
    diagonal(vehicle.inertia_inv, 50.0, 40.0, 25.0);

    cs_rotor_config_t control_rotors[4];
    configure_rotor(&vehicle.rotors[0], &control_rotors[0], arm, 0.0, 1.0);
    configure_rotor(&vehicle.rotors[1], &control_rotors[1], 0.0, arm, -1.0);
    configure_rotor(&vehicle.rotors[2], &control_rotors[2], -arm, 0.0, 1.0);
    configure_rotor(&vehicle.rotors[3], &control_rotors[3], 0.0, -arm, -1.0);

    dm_vehicle_model_t plant;
    memset(&plant, 0, sizeof(plant));
    plant.config = &vehicle;
    const double initial_roll = 12.0 * (3.14159265358979323846 / 180.0);
    plant.state.quaternion[0] = cos(0.5 * initial_roll);
    plant.state.quaternion[1] = sin(0.5 * initial_roll);
    assert(dm_vehicle_config_validate(&vehicle) == DM_OK);

    cs_mixer_t mixer;
    assert(cs_mixer_init(&mixer, control_rotors, 4) == 0);

    const cs_pid_gains_t gains[3] = {
        {.kp = 0.12, .ki = 0.0, .kd = 0.035,
         .integrator_limit = 0.0, .output_limit = 0.25},
        {.kp = 0.12, .ki = 0.0, .kd = 0.035,
         .integrator_limit = 0.0, .output_limit = 0.25},
        {.kp = 0.08, .ki = 0.0, .kd = 0.025,
         .integrator_limit = 0.0, .output_limit = 0.15},
    };
    cs_attitude_pid_t controller;
    assert(cs_attitude_pid_init_checked(&controller, gains, 1.0));

    const ins_config_t ins_config = {
        .nominal_dt = dt,
        .min_dt = 0.001,
        .max_dt = 0.004,
        .complementary_alpha = 0.98,
    };
    ins_t ins;
    assert(ins_init_checked(&ins, &ins_config) == INS_OK);

    const cs_attitude_setpoint_t target = {
        .quaternion = {1.0, 0.0, 0.0, 0.0},
        .angular_rate = {0.0, 0.0, 0.0},
    };
    const double gravity_ned[3] = {0.0, 0.0, 1.0};
    const double magnetic_ned[3] = {0.9363291775690445, 0.0,
                                    0.3511234415883917};
    const double initial_error = attitude_angle(plant.state.quaternion);
    double maximum_estimation_error = 0.0;

    for (int step = 0; step < 2500; ++step) {
        ins_sensor_sample_t sample;
        memset(&sample, 0, sizeof(sample));
        sample.timestamp = step * dt;
        memcpy(sample.gyro, plant.state.angular_rate, sizeof(sample.gyro));
        body_vector(plant.state.quaternion, gravity_ned, sample.accel);
        body_vector(plant.state.quaternion, magnetic_ned, sample.mag);
        sample.mag_valid = 1;
        assert(ins_update_checked(&ins, &sample) == INS_OK);

        ins_output_t estimate;
        assert(ins_get_output_checked(&ins, &estimate) == INS_OK);
        double inverse_estimate[4];
        double estimation_delta[4];
        assert(quaternion_inverse(estimate.quaternion, inverse_estimate));
        quaternion_multiply(inverse_estimate, plant.state.quaternion,
                            estimation_delta);
        const double estimation_error = attitude_angle(estimation_delta);
        if (estimation_error > maximum_estimation_error) {
            maximum_estimation_error = estimation_error;
        }

        cs_state_t feedback;
        memset(&feedback, 0, sizeof(feedback));
        memcpy(feedback.quaternion, estimate.quaternion,
               sizeof(feedback.quaternion));
        memcpy(feedback.angular_rate, estimate.angular_rate,
               sizeof(feedback.angular_rate));

        cs_actuator_command_t command;
        assert(cs_attitude_pid_update_checked(
            &controller, &target, &feedback, dt, &command));
        command.collective_thrust = -mass * vehicle.gravity;

        double rotor_omega[DM_MAX_ROTORS] = {0.0};
        assert(cs_mixer_mix(&mixer, &command, rotor_omega) == 0);
        assert(dm_vehicle_step_rk4_checked(&plant, rotor_omega, dt) == DM_OK);
    }

    const double final_error = attitude_angle(plant.state.quaternion);
    printf("closed-loop attitude: initial=%.6f rad final=%.6f rad "
           "max_estimation=%.6f rad\n",
           initial_error, final_error, maximum_estimation_error);
    assert(final_error < initial_error * 0.08);
    assert(maximum_estimation_error < 0.01);
    return 0;
}
