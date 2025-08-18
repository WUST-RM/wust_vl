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

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <vector>

class PnPSolver {
public:
    PnPSolver(cv::SolvePnPMethod method = cv::SOLVEPNP_IPPE);

    // Set an object coord system
    void setObjectPoints(
        const std::string& coord_frame_name,
        const std::vector<cv::Point3f>& object_points
    ) noexcept;

    // Get 3d position of the object coord system using PnP algorithm
    template<class InputArray>
    bool solvePnPGeneric(
        const InputArray& image_points,
        std::vector<cv::Mat>& rvecs,
        std::vector<cv::Mat>& tvecs,
        const std::string& coord_frame_name,
        const cv::Mat& camera_intrinsic,
        const cv::Mat& camera_distortion
    ) {
        rvecs.clear();
        tvecs.clear();
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto& object_points = object_points_map_[coord_frame_name];
            int solutions = cv::solvePnPGeneric(
                object_points,
                image_points,
                camera_intrinsic,
                camera_distortion,
                rvecs,
                tvecs,
                false,
                method_
            );
            return solutions > 0;
        } else {
            std::cout << "No object points found for coord frame: " << coord_frame_name
                      << std::endl;
            return false;
        }
    }

    // Get 3d position of the object coord system using PnP algorithm
    template<class InputArray>
    bool solvePnP(
        const InputArray& image_points,
        cv::Mat& rvec,
        cv::Mat& tvec,
        const std::string& coord_frame_name,
        const cv::Mat& camera_intrinsic,
        const cv::Mat& camera_distortion
    ) {
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto& object_points = object_points_map_[coord_frame_name];
            return cv::solvePnP(
                object_points,
                image_points,
                camera_intrinsic,
                camera_distortion,
                rvec,
                tvec,
                false,
                method_
            );
        } else {
            return false;
        }
    }
    std::vector<cv::Point2f> getImagePoints(
        const cv::Mat& rvec,
        const cv::Mat& tvec,
        const std::string& coord_frame_name,
        const cv::Mat& camera_intrinsic,
        const cv::Mat& camera_distortion
    ) {
        std::vector<cv::Point2f> image_points;
        if (object_points_map_.find(coord_frame_name) == object_points_map_.end()) {
            throw std::runtime_error("Unknown coord_frame_name");
        }
        const auto& object_points = object_points_map_[coord_frame_name];

        cv::projectPoints(
            object_points,
            rvec,
            tvec,
            camera_intrinsic,
            camera_distortion,
            image_points
        );

        return image_points;
    }

    // Calculate the distance between armor center and image center
    float calculateDistanceToCenter(const cv::Point2f& image_point, const cv::Mat& camera_intrinsic)
        const noexcept;

    double calculateReprojectionError(
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& rvec,
        const cv::Mat& tvec,
        const std::string& coord_frame_name,
        const cv::Mat& camera_intrinsic,
        const cv::Mat& camera_distortion
    ) const noexcept;

private:
    std::unordered_map<std::string, std::vector<cv::Point3f>> object_points_map_;

    cv::SolvePnPMethod method_;
};
