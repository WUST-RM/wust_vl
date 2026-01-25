#pragma once
#include <cmath>
namespace wust_vl {
namespace common {
    namespace utils {
        inline double interpolate_sin_rad(double theta_rad, double val0, double val90) {
            return val0 + (val90 - val0) * std::sin(theta_rad);
        }

        // 正弦插值（角度输入）
        inline double interpolate_sin_deg(double theta_deg, double val0, double val90) {
            double theta_rad = theta_deg * M_PI / 180.0;
            return interpolate_sin_rad(theta_rad, val0, val90);
        }

        inline double interpolate_cos_rad(double theta_rad, double val0, double val90) {
            return val90 + (val0 - val90) * std::cos(theta_rad);
        }

        // 余弦插值（角度输入）
        inline double interpolate_cos_deg(double theta_deg, double val0, double val90) {
            double theta_rad = theta_deg * M_PI / 180.0;
            return interpolate_cos_rad(theta_rad, val0, val90);
        }
    } // namespace utils
} // namespace common
} // namespace wust_vl