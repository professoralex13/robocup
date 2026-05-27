#include <numbers>

float diff_angle(float angle1, float angle2) {
    float output = angle2 - angle1;

    if (output > std::numbers::pi) {
        output -= 2 * std::numbers::pi;
    }

    if (output <= -std::numbers::pi) {
        output += 2 * std::numbers::pi;
    }

    return output;
}

float wrap_heading(float heading) {
    if (heading >= 2 * std::numbers::pi) {
        heading -= 2 * std::numbers::pi;
    }

    if (heading < 0) {
        heading += 2 * std::numbers::pi;
    }

    return heading;
}