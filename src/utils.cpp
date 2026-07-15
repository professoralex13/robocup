#include "utils.hpp"

using Eigen::Vector2f;

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

Vector2f get_closest_point(Vector2f point, Vector2f start, Vector2f end) {
    auto v = end - start;
    auto d = point - start;

    float t = v.dot(d) / v.dot(v);

    t = std::clamp(t, 0.0F, 1.0F);

    return start + v * t;
}

Vector2f get_snapped_radius_target(Vector2f center, float radius, Vector2f start, Vector2f end) {
    auto v = end - start;
    auto d = start - center;

    float a = v.dot(v);
    float b = 2 * (d.dot(v));
    float c = d.dot(d) - radius * radius;

    float discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        // When the circle intersects the line extension, return the furthest along intersection
        float t = (-b + sqrt(discriminant)) / (2.0 * a);

        // Snap the result to along the line segment
        t = std::clamp(t, 0.0F, 1.0F);

        return start + v * t;
    }

    // If the line extension does not intersect the circle, fallback to closest_point (this also
    // snaps to the line segment)
    return get_closest_point(center, start, end);
}