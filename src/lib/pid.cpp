#include "lib/pid.hpp"
#include <algorithm>
#include <cmath>
#include <core_pins.h>
#include <optional>

PIDController::PIDController(float kp, float ki, float kd, float stable_error)
    : kp(kp), ki(ki), kd(kd), stable_error(stable_error) {}

float PIDController::update(float error) {
    check_stability(error);

    long timestamp = micros();

    float delta_t;

    if (last_timestamp.has_value()) {
        delta_t = (timestamp - last_timestamp.value()) * 1e-6;
    } else {
        delta_t = 0;
    }

    integral_sum += error * delta_t;

    // If the controller has specified integral bounds, check that the error is
    // within those bounds, otherwise the integral sum is set to zero
    if (integral_bounds.has_value()) {
        auto [lower, upper] = integral_bounds.value();

        if (error < lower || error > upper) {
            integral_sum = 0;
        }
    }

    float output = 0;
    output += kp * error;
    output += ki * integral_sum;

    if (delta_t != 0) {
        output += kd * (error - previous_error) / delta_t;
    }

    // Clamp the output value to be between the two specified output limits (if
    // they exist)
    if (output_limits.has_value()) {
        auto [min, max] = output_limits.value();
        output = std::clamp(output, min, max);
    }

    previous_error = error;
    last_timestamp = timestamp;

    return output;
}

PIDController PIDController::with_output_limits(float min, float max) {
    output_limits = {min, max};

    return *this;
}

PIDController PIDController::with_integral_bounds(float lower, float upper) {
    integral_bounds = {lower, upper};

    return *this;
}

bool PIDController::check_stability(float error) {
    stablized = (fabs(error - previous_error) <= stable_error * 0.1 && fabs(error) <= stable_error);

    return stablized;
}

bool PIDController::is_stable() const { return stablized; }