// Created by Chengfu Zou on 2024.1.19
// Copyright(C) FYT Vision Group. All rights resevred.
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

#include "algorithm/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

PnPSolver::PnPSolver(cv::SolvePnPMethod method): method_(method) {}

void PnPSolver::setObjectPoints(
    const std::string& coord_frame_name,
    const std::vector<cv::Point3f>& object_points
) noexcept {
    object_points_map_[coord_frame_name] = object_points;
}

float PnPSolver::calculateDistanceToCenter(
    const cv::Point2f& image_point,
    const cv::Mat& camera_intrinsic
) const noexcept {
    float cx = camera_intrinsic.at<double>(0, 2);
    float cy = camera_intrinsic.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

double PnPSolver::calculateReprojectionError(
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const std::string& coord_frame_name,
    const cv::Mat& camera_intrinsic,
    const cv::Mat& camera_distortion
) const noexcept {
    if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
        const auto& object_points = object_points_map_.at(coord_frame_name);
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(
            object_points,
            rvec,
            tvec,
            camera_intrinsic,
            camera_distortion,
            reprojected_points
        );
        double error = 0;
        for (size_t i = 0; i < image_points.size(); ++i) {
            error += cv::norm(image_points[i] - reprojected_points[i]);
        }
        return error;
    } else {
        return 0;
    }
}
