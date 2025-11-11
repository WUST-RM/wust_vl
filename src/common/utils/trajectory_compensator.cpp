// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "common/utils/trajectory_compensator.hpp"

bool TrajectoryCompensator::compensate(
    const Eigen::Vector3d& target_position,
    double& pitch,
    const double bullet_speed
) const noexcept {
    double target_height = target_position(2);
    // The iterative_height is used to calculate angle in each iteration
    double iterative_height = target_height;
    double impact_height = 0;
    double distance = std::sqrt(
        target_position(0) * target_position(0) + target_position(1) * target_position(1)
    );
    double angle = std::atan2(target_height, distance);
    double dh = 0;
    // Iterate to find the right angle, which makes the impact height equal to the
    // target height
    for (int i = 0; i < iteration_times_; ++i) {
        angle = std::atan2(iterative_height, distance);
        if (std::abs(angle) > M_PI / 2.5) {
            break;
        }
        impact_height = calculateTrajectory(distance, angle, bullet_speed);
        dh = target_height - impact_height;
        if (std::abs(dh) < 0.01) {
            break;
        }
        iterative_height += dh;
    }
    if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
        return false;
    }
    pitch = angle;
    return true;
}

std::vector<std::pair<double, double>>
TrajectoryCompensator::getTrajectory(double distance, double angle, const double bullet_speed)
    const noexcept {
    std::vector<std::pair<double, double>> trajectory;

    if (distance < 0) {
        return trajectory;
    }

    for (double x = 0; x < distance; x += 0.03) {
        trajectory.emplace_back(x, calculateTrajectory(x, angle, bullet_speed));
    }
    return trajectory;
}

double
IdealCompensator::calculateTrajectory(const double x, const double angle, const double bullet_speed)
    const noexcept {
    double t = x / (bullet_speed * cos(angle));
    double y = bullet_speed * sin(angle) * t - 0.5 * gravity_ * t * t;
    return y;
}

double IdealCompensator::getFlyingTime(
    const Eigen::Vector3d& target_position,
    const double bullet_speed
) const noexcept {
    double distance =
        sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
    double angle = atan2(target_position(2), distance);
    double t = distance / (bullet_speed * cos(angle));

    return t;
}

double ResistanceCompensator::calculateTrajectory(
    const double x,
    const double angle,
    const double bullet_speed
) const noexcept {
    double r = resistance_ < 1e-4 ? 1e-4 : resistance_;
    double t = (exp(r * x) - 1) / (r * bullet_speed * cos(angle));
    double y = bullet_speed * sin(angle) * t - 0.5 * gravity_ * t * t;
    return y;
}

double ResistanceCompensator::getFlyingTime(
    const Eigen::Vector3d& target_position,
    const double bullet_speed
) const noexcept {
    double r = resistance_ < 1e-4 ? 1e-4 : resistance_;
    double distance =
        sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
    double angle = atan2(target_position(2), distance);
    double t = (exp(r * distance) - 1) / (r * bullet_speed * cos(angle));

    return t;
}

double RK4Compensator::calculateTrajectory(
    const double target_x,
    const double angle,
    const double bullet_speed
) const noexcept {
    const double k = 0.5 * rho_ * Cd_ * A_ / mass_;
    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    const double g = gravity_;

    double px = 0.0, pz = 0.0;
    double vx = bullet_speed * cos_a;
    double vz = bullet_speed * sin_a;

    double px_prev = 0.0, pz_prev = 0.0;
    double dt = 0.005;
    const double base_dt = 0.005;
    const int max_steps = 1000000;
    int steps = 0;

    while (px < target_x && pz > -50.0 && steps < max_steps) {
        px_prev = px;
        pz_prev = pz;

        double speed = std::sqrt(vx * vx + vz * vz);
        double drag = (speed > 1e-9) ? -k * speed : 0.0;

        double ax = drag * vx;
        double az = drag * vz - g;

        double vx_mid = vx + 0.5 * dt * ax;
        double vz_mid = vz + 0.5 * dt * az;

        double speed_mid = std::sqrt(vx_mid * vx_mid + vz_mid * vz_mid);
        double drag_mid = (speed_mid > 1e-9) ? -k * speed_mid : 0.0;

        double ax_mid = drag_mid * vx_mid;
        double az_mid = drag_mid * vz_mid - g;

        px += dt * vx_mid;
        pz += dt * vz_mid;
        vx += dt * ax_mid;
        vz += dt * az_mid;

        ++steps;

        double remain_x = target_x - px;
        double distance_ratio = std::clamp(remain_x / target_x, 0.05, 1.0);
        double speed_ratio = std::clamp(speed / (bullet_speed + 1e-9), 0.3, 1.5);

        dt = std::clamp(
            base_dt * (0.8 + 0.4 * speed_ratio) * (0.5 + 0.5 * distance_ratio),
            0.001,
            0.006
        );

        if (vz < -1e-2 && pz < 0.0)
            break;
        if (px > target_x + 2.0)
            break;
    }

    const double x0 = px_prev, x1 = px;
    const double z0 = pz_prev, z1 = pz;
    if (std::abs(x1 - x0) < 1e-9)
        return z1;
    const double alpha = std::clamp((target_x - x0) / (x1 - x0), 0.0, 1.0);
    return z0 + alpha * (z1 - z0);
}

double RK4Compensator::getFlyingTime(
    const Eigen::Vector3d& target_position,
    const double bullet_speed
) const noexcept {
    const double distance = std::hypot(target_position(0), target_position(1));
    const double angle = std::atan2(target_position(2), distance);

    const double k = 0.5 * rho_ * Cd_ * A_ / mass_;
    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    const double g = gravity_;

    double px = 0.0, pz = 0.0;
    double vx = bullet_speed * cos_a;
    double vz = bullet_speed * sin_a;

    double dt = 0.005;
    const double base_dt = 0.005;
    const int max_steps = 1000000;
    double t = 0.0;
    int steps = 0;

    while (px < distance && pz > -50.0 && steps < max_steps) {
        double speed = std::sqrt(vx * vx + vz * vz);
        double drag = (speed > 1e-9) ? -k * speed : 0.0;

        double ax = drag * vx;
        double az = drag * vz - g;

        // RK2 Midpoint
        double vx_mid = vx + 0.5 * dt * ax;
        double vz_mid = vz + 0.5 * dt * az;

        double speed_mid = std::sqrt(vx_mid * vx_mid + vz_mid * vz_mid);
        double drag_mid = (speed_mid > 1e-9) ? -k * speed_mid : 0.0;

        double ax_mid = drag_mid * vx_mid;
        double az_mid = drag_mid * vz_mid - g;

        px += dt * vx_mid;
        pz += dt * vz_mid;
        vx += dt * ax_mid;
        vz += dt * az_mid;

        t += dt;
        ++steps;

        double remain_x = distance - px;
        double distance_ratio = std::clamp(remain_x / distance, 0.05, 1.0);
        double speed_ratio = std::clamp(speed / (bullet_speed + 1e-9), 0.3, 1.5);

        dt = std::clamp(
            base_dt * (0.8 + 0.4 * speed_ratio) * (0.5 + 0.5 * distance_ratio),
            0.001,
            0.006
        );

        if (vz < -1e-2 && pz < 0.0)
            break;
        if (px > distance + 2.0)
            break;
    }

    return t;
}
