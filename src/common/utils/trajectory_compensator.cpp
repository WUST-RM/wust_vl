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
namespace wust_vl {
namespace common {
    namespace utils {
        bool TrajectoryCompensator::compensate(
            const Eigen::Vector3d& target_position,
            double& pitch,
            const double bullet_speed
        ) const noexcept {
            const double target_height = target_position(2);

            const double distance = std::sqrt(
                target_position(0) * target_position(0) + target_position(1) * target_position(1)
            );

            if (distance < 1e-6 || bullet_speed < 1e-3) {
                return false;
            }

            auto f = [&](double angle) {
                double y = calculateTrajectory(distance, angle, bullet_speed);
                return y - target_height;
            };

            double left = -M_PI / 4.0; // -45°
            double right = M_PI / 3.0; // 60°

            double f_left = f(left);
            double f_right = f(right);

            if (f_left * f_right > 0) {
                return false;
            }

            double mid = 0;
            double f_mid = 0;

            for (int i = 0; i < iteration_times_; ++i) {
                mid = 0.5 * (left + right);
                f_mid = f(mid);

                if (std::abs(f_mid) < 0.01) {
                    pitch = mid;
                    return true;
                }

                if (f_left * f_mid < 0) {
                    right = mid;
                    f_right = f_mid;
                } else {
                    left = mid;
                    f_left = f_mid;
                }
            }
            pitch = mid;
            return true;
        }

        std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
            double distance,
            double angle,
            const double bullet_speed
        ) const noexcept {
            std::vector<std::pair<double, double>> trajectory;

            if (distance < 0) {
                return trajectory;
            }

            for (double x = 0; x < distance; x += 0.03) {
                trajectory.emplace_back(x, calculateTrajectory(x, angle, bullet_speed));
            }
            return trajectory;
        }

        double IdealCompensator::calculateTrajectory(
            const double x,
            const double angle,
            const double bullet_speed
        ) const noexcept {
            double t = x / (bullet_speed * cos(angle));
            double y = bullet_speed * sin(angle) * t - 0.5 * gravity_ * t * t;
            return y;
        }

        double IdealCompensator::getFlyingTime(
            const Eigen::Vector3d& target_position,
            const double bullet_speed
        ) const noexcept {
            double distance = sqrt(
                target_position(0) * target_position(0) + target_position(1) * target_position(1)
            );
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
            double distance = sqrt(
                target_position(0) * target_position(0) + target_position(1) * target_position(1)
            );
            double angle = atan2(target_position(2), distance);
            double t = (exp(r * distance) - 1) / (r * bullet_speed * cos(angle));

            return t;
        }
        double K1Compensator ::getFlyingTime(
            const Eigen::Vector3d& target_position,
            const double bullet_speed
        ) const noexcept {
            double distance = std::sqrt(
                target_position(0) * target_position(0) + target_position(1) * target_position(1)
            );
            double angle = std::atan2(target_position(2), distance);
            double k1 = k1_;
            double t = (std::exp(k1 * distance) - 1) / (k1 * bullet_speed * std::cos(angle));
            return t;
        }
        double K1Compensator ::calculateTrajectory(
            const double x,
            const double angle,
            const double bullet_speed
        ) const noexcept {
            double k1 = k1_;
            double t = (std::exp(k1 * x) - 1) / (k1 * bullet_speed * std::cos(angle));
            double y = bullet_speed * std::sin(angle) * t - 0.5 * gravity_ * t * t;
            return y;
        }
    } // namespace utils
} // namespace common
} // namespace wust_vl