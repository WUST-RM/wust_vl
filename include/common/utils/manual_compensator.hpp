// Copyright (C) FYT Vision Group. All rights reserved.
// Copyright 2025 Xiaojian Wu
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

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

constexpr size_t NORMAL_STR_NUM = 6;

struct OffsetEntry {
    double d_min, d_max;
    double h_min, h_max;
    double pitch_off, yaw_off;
};

class LineRegion {
public:
    LineRegion(const double l, const double u): lowwer_(l), upper_(u) {}

    bool checkPoint(const double p) const {
        if (p > lowwer_ && p < upper_) {
            return true;
        }
        return false;
    }

    bool checkIntersection(const LineRegion& l) const {
        if (checkPoint(l.lowwer_) || checkPoint(l.upper_)) {
            return true;
        }
        return false;
    }

private:
    double lowwer_;
    double upper_;
};

class ManualCompensator {
public:
    struct HeightMapNode {
        HeightMapNode(const LineRegion& region, const double pitch, const double yaw):
            height_region(region),
            pitch_offset(pitch),
            yaw_offset(yaw) {}
        LineRegion height_region;
        double pitch_offset;
        double yaw_offset;
    };

    struct DistMapNode {
        DistMapNode(const LineRegion& region, const std::vector<HeightMapNode> h_nodes):
            dist_region(region),
            height_map(h_nodes) {}
        LineRegion dist_region;
        std::vector<HeightMapNode> height_map;
    };

    ManualCompensator() = default;

    std::vector<double> angleHardCorrect(const double dist, const double height);

    bool updateMap(
        const LineRegion& d_region,
        const LineRegion& h_region,
        const double pitch_offset,
        const double yaw_offset
    );

    bool updateMapByStr(const std::string& str);

    bool updateMapFlow(const std::vector<std::string> strs) {
        for (const auto& str: strs) {
            if (!updateMapByStr(str)) {
                return false;
            }
        }
        return true;
    }
    bool updateMapFlow(const std::vector<std::vector<double>>& nums_list) {
        for (const auto& nums: nums_list) {
            if (nums.size() < 6)
                return false;
            LineRegion d_region(nums[0], nums[1]);
            LineRegion h_region(nums[2], nums[3]);
            if (!updateMap(d_region, h_region, nums[4], nums[5])) {
                return false;
            }
        }
        return true;
    }
    bool updateMapFlow(const std::vector<OffsetEntry>& entries) {
        for (const auto& e: entries) {
            if (!updateMap(
                    LineRegion(e.d_min, e.d_max),
                    LineRegion(e.h_min, e.h_max),
                    e.pitch_off,
                    e.yaw_off
                )) {
                return false;
            }
        }
        return true;
    }
    std::tuple<double, double>
    applyManualCompensator(double distance, double z, double raw_yaw, double raw_pitch) {
        auto offs = this->angleHardCorrect(distance, z);
        double yaw_off = offs[1] * M_PI / 180.0;
        double pitch_off = offs[0] * M_PI / 180.0;
        return { raw_yaw + yaw_off, raw_pitch + pitch_off };
    }

private:
    bool parseStr(const std::string& str, std::vector<double>& nums);

    std::vector<DistMapNode> angle_offset_map_;
};