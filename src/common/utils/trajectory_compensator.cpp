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

double RK4Compensator::calculateTrajectory(const double target_x, const double angle, const double bullet_speed)
    const noexcept {
    // 使用基类中的物理参数
    double k = 0.5 * rho_ * Cd_ * A_ / mass_;

    Eigen::Vector3d p(0.0, 0.0, 0.0);
    Eigen::Vector3d v(bullet_speed * std::cos(angle), 0.0, bullet_speed * std::sin(angle));

    double dt = 0.001;
    int max_steps = 5e6;
    Eigen::Vector3d p_prev = p, v_prev = v;

    auto accel = [&](const Eigen::Vector3d& vel) -> Eigen::Vector3d {
        double speed = vel.norm();
        if (speed < 1e-9) return Eigen::Vector3d(0, 0, -gravity_);
        Eigen::Vector3d drag = -k * speed * vel;
        return drag + Eigen::Vector3d(0, 0, -gravity_);
    };

    int steps = 0;
    while (p.x() < target_x && p.z() > -100 && steps < max_steps) {
        // RK4 积分
        Eigen::Vector3d k1_v = accel(v);
        Eigen::Vector3d k1_p = v;

        Eigen::Vector3d v2 = v + 0.5 * dt * k1_v;
        Eigen::Vector3d p2 = p + 0.5 * dt * k1_p;
        Eigen::Vector3d k2_v = accel(v2);
        Eigen::Vector3d k2_p = v2;

        Eigen::Vector3d v3 = v + 0.5 * dt * k2_v;
        Eigen::Vector3d p3 = p + 0.5 * dt * k2_p;
        Eigen::Vector3d k3_v = accel(v3);
        Eigen::Vector3d k3_p = v3;

        Eigen::Vector3d v4 = v + dt * k3_v;
        Eigen::Vector3d p4 = p + dt * k3_p;
        Eigen::Vector3d k4_v = accel(v4);
        Eigen::Vector3d k4_p = v4;

        p_prev = p;
        v_prev = v;

        p += dt * (k1_p + 2*k2_p + 2*k3_p + k4_p) / 6.0;
        v += dt * (k1_v + 2*k2_v + 2*k3_v + k4_v) / 6.0;

        steps++;
    }

    double x0 = p_prev.x(), x1 = p.x();
    double z0 = p_prev.z(), z1 = p.z();
    if (x1 == x0) return z1;
    double alpha = (target_x - x0) / (x1 - x0);
    alpha = std::clamp(alpha, 0.0, 1.0);
    return z0 + alpha * (z1 - z0);
}

double RK4Compensator::getFlyingTime(
    const Eigen::Vector3d& target_position,
    const double bullet_speed
) const noexcept {
    double distance = std::sqrt(target_position(0)*target_position(0) + target_position(1)*target_position(1));
    double angle = std::atan2(target_position(2), distance);

    double k = 0.5 * rho_ * Cd_ * A_ / mass_;

    Eigen::Vector3d p(0, 0, 0);
    Eigen::Vector3d v(bullet_speed * std::cos(angle), 0, bullet_speed * std::sin(angle));

    double dt = 0.001;
    double t = 0.0;

    auto accel = [&](const Eigen::Vector3d& vel) -> Eigen::Vector3d {
        double speed = vel.norm();
        if (speed < 1e-9) return Eigen::Vector3d(0, 0, -gravity_);
        Eigen::Vector3d drag = -k * speed * vel;
        return drag + Eigen::Vector3d(0, 0, -gravity_);
    };

    while (p.x() < distance && p.z() > -100) {
        Eigen::Vector3d k1_v = accel(v);
        Eigen::Vector3d k1_p = v;

        Eigen::Vector3d v2 = v + 0.5 * dt * k1_v;
        Eigen::Vector3d p2 = p + 0.5 * dt * k1_p;
        Eigen::Vector3d k2_v = accel(v2);
        Eigen::Vector3d k2_p = v2;

        Eigen::Vector3d v3 = v + 0.5 * dt * k2_v;
        Eigen::Vector3d p3 = p + 0.5 * dt * k2_p;
        Eigen::Vector3d k3_v = accel(v3);
        Eigen::Vector3d k3_p = v3;

        Eigen::Vector3d v4 = v + dt * k3_v;
        Eigen::Vector3d p4 = p + dt * k3_p;
        Eigen::Vector3d k4_v = accel(v4);
        Eigen::Vector3d k4_p = v4;

        p += dt * (k1_p + 2*k2_p + 2*k3_p + k4_p) / 6.0;
        v += dt * (k1_v + 2*k2_v + 2*k3_v + k4_v) / 6.0;
        t += dt;
    }

    return t;
}


