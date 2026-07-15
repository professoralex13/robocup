#pragma once

#include "eigen.h"

using Eigen::Vector2f;

float diff_angle(float angle1, float angle2);
float wrap_heading(float heading);

Vector2f get_closest_point(Vector2f point, Vector2f start, Vector2f end);

/**
 * Calculates the target point for a robot to aim towards during pure pursuit.
 * If the line is within the given distance `radius` of `center`, the result will be the circle
 * intersection that is closest to `end` If it is further away than `radius`, the result will be the
 * closest point on the line segment to `center`
 *
 * The result is snapped to the line segment between start and end.
 */
Vector2f get_snapped_radius_target(Vector2f center, float radius, Vector2f start, Vector2f end);