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

#pragma once
#include <Eigen/Dense>
#include <memory>
#include <tuple>
class TrajectoryCompensator {
public:
    TrajectoryCompensator() = default;
    virtual ~TrajectoryCompensator() = default;

    // Compensate the trajectory of the bullet, return the pitch increment
    bool
    compensate(const Eigen::Vector3d& target_position, double& pitch, const double bullet_speed)
        const noexcept;

    virtual double getFlyingTime(const Eigen::Vector3d& target_position, const double bullet_speed)
        const noexcept = 0;

    std::vector<std::pair<double, double>>
    getTrajectory(double distance, double angle, const double bullet_speed) const noexcept;

    int iteration_times_ = 20;
    double gravity_ = 9.8;
    double resistance_ = 0.01;



protected:
    // Calculate the trajectory of the bullet, return the vertical impact point
    virtual double
    calculateTrajectory(const double x, const double angle, const double bullet_speed)
        const noexcept = 0;
};

// IdealCompensator does not consider the air resistance
class IdealCompensator: public TrajectoryCompensator {
public:
    double getFlyingTime(const Eigen::Vector3d& target_position, const double bullet_speed)
        const noexcept override;

protected:
    double calculateTrajectory(const double x, const double angle, const double bullet_speed)
        const noexcept override;
};

// ResistanceCompensator considers the air resistance
class ResistanceCompensator: public TrajectoryCompensator {
public:
    double getFlyingTime(const Eigen::Vector3d& target_position, const double bullet_speed)
        const noexcept override;

protected:
    double calculateTrajectory(const double x, const double angle, const double bullet_speed)
        const noexcept override;
};



// Factory class for trajectory compensator
class CompensatorFactory: public TrajectoryCompensator {
public:
    static std::shared_ptr<TrajectoryCompensator> createCompensator(const std::string& type) {
        if (type == "ideal") {
            return std::make_shared<IdealCompensator>();
        } else if (type == "resistance") {
            return std::make_shared<ResistanceCompensator>();
        }else {
            return nullptr;
        }
    }

private:
    CompensatorFactory() = delete;
};
