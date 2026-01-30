#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <vector>
namespace wust_vl {
namespace common {
    namespace utils {
        template<typename T, typename Enable = void>
        struct MotionTraits {
            static void unwrap(const T& /*prev*/, T& /*curr*/) {}
            static T interpolate(const T& a, const T& b, double t) {
                return a + (b - a) * t; // 默认线性插值
            }
        };

        template<>
        struct MotionTraits<Eigen::Quaterniond> {
            static void unwrap(const Eigen::Quaterniond& prev, Eigen::Quaterniond& curr) {
                if (prev.dot(curr) < 0.0)
                    curr.coeffs() *= -1.0;
            }

            static Eigen::Quaterniond
            interpolate(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double t) {
                return a.slerp(t, b);
            }
        };

        template<typename T, typename = void>
        struct has_motion_traits: std::false_type {};

        template<typename T>
        struct has_motion_traits<
            T,
            std::void_t<
                decltype(MotionTraits<T>::unwrap(std::declval<const T&>(), std::declval<T&>())),
                decltype(MotionTraits<
                         T>::interpolate(std::declval<const T&>(), std::declval<const T&>(), 0.0)
                )>>: std::true_type {};

        template<typename T, size_t BUFFER_SIZE = 512>
        class MotionBufferGeneric {
            static_assert(
                has_motion_traits<T>::value,
                "MotionTraits<T> must exist with unwrap() and interpolate()"
            );

        public:
            using TimePoint = std::chrono::steady_clock::time_point;

            struct Stamped {
                T data;
                TimePoint stamp;
            };

            void push(T data, TimePoint stamp) {
                std::unique_lock lock(mutex_);
                if (has_last_) {
                    MotionTraits<T>::unwrap(last_.data, data);
                }

                buffer_[head_] = { data, stamp };
                time_buffer_[head_] = stamp;

                head_ = (head_ + 1) % BUFFER_SIZE;
                if (size_ < BUFFER_SIZE)
                    ++size_;

                last_ = buffer_[(head_ + BUFFER_SIZE - 1) % BUFFER_SIZE];
                has_last_ = true;
            }

            std::optional<Stamped> get_interpolated(TimePoint t_query) const {
                std::shared_lock lock(mutex_);
                if (size_ < 2)
                    return std::nullopt;

                size_t begin = (head_ + BUFFER_SIZE - size_) % BUFFER_SIZE;

                std::vector<TimePoint> times(size_);
                for (size_t i = 0; i < size_; ++i)
                    times[i] = time_buffer_[(begin + i) % BUFFER_SIZE];

                auto it_hi = std::lower_bound(times.begin(), times.end(), t_query);
                if (it_hi == times.begin() || it_hi == times.end())
                    return std::nullopt;

                size_t idx_hi = std::distance(times.begin(), it_hi);
                size_t idx_lo = idx_hi - 1;

                const auto& lo = buffer_[(begin + idx_lo) % BUFFER_SIZE];
                const auto& hi = buffer_[(begin + idx_hi) % BUFFER_SIZE];

                double span = std::chrono::duration<double>(hi.stamp - lo.stamp).count();
                if (span <= 0.0)
                    return lo;

                double t = std::chrono::duration<double>(t_query - lo.stamp).count() / span;

                Stamped res;
                res.data = MotionTraits<T>::interpolate(lo.data, hi.data, t);
                res.stamp = t_query;
                return res;
            }

            std::optional<Stamped> get_last() const {
                std::shared_lock lock(mutex_);
                if (!has_last_)
                    return std::nullopt;
                return last_;
            }

        private:
            std::array<Stamped, BUFFER_SIZE> buffer_;
            std::array<TimePoint, BUFFER_SIZE> time_buffer_;
            size_t head_ = 0, size_ = 0;

            bool has_last_ = false;
            Stamped last_;

            mutable std::shared_mutex mutex_;
        };
    } // namespace utils
} // namespace common
} // namespace wust_vl