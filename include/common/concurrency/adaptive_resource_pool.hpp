#pragma once

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

/// A movable wrapper around std::atomic<bool>.
/// Standard std::atomic is neither copyable nor movable, so this
/// helper allows moving by copying the value in a relaxed manner.
struct MovableAtomicBool {
    std::atomic<bool> v;

    explicit MovableAtomicBool(bool b = false) noexcept: v(b) {}

    bool load(std::memory_order m = std::memory_order_seq_cst) const noexcept {
        return v.load(m);
    }

    void store(bool b, std::memory_order m = std::memory_order_seq_cst) noexcept {
        v.store(b, m);
    }

    bool exchange(bool b, std::memory_order m = std::memory_order_seq_cst) noexcept {
        return v.exchange(b, m);
    }

    // Move constructor: copies the current value
    MovableAtomicBool(MovableAtomicBool&& o) noexcept: v(o.v.load(std::memory_order_relaxed)) {}

    // Move assignment: copies the current value
    MovableAtomicBool& operator=(MovableAtomicBool&& o) noexcept {
        v.store(o.v.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }

    MovableAtomicBool(const MovableAtomicBool&) = delete;
    MovableAtomicBool& operator=(const MovableAtomicBool&) = delete;
};

/// AdaptiveResourcePool manages a pool of reusable resources (e.g., connections, buffers).
/// It can release unused resources and restore them later based on provided strategies.
template<typename T>
class AdaptiveResourcePool {
public:
    struct Params {
        /// Initializes the initial set of resources.
        std::function<std::vector<std::unique_ptr<T>>()> resource_initializer;

        /// Determines whether resources should be restored based on active count.
        std::function<bool(size_t)> can_restore;

        /// Determines whether resources should be released based on active count.
        std::function<bool(size_t)> should_release;

        /// Restores a resource at the given index.
        std::function<std::unique_ptr<T>(size_t)> restore_func;

        /// Releases a given resource.
        std::function<void(std::unique_ptr<T>&)> release_func;

        /// Optional logging function.
        std::function<void(const std::string&)> logger = [](const std::string&) {};
    };

    /// Constructs the resource pool using the given parameters.
    explicit AdaptiveResourcePool(const Params& params): params_(params) {
        resources_ = params.resource_initializer();
        busy_.resize(resources_.size());
        released_.resize(resources_.size(), false);
    }

    /// Cleans up all resources upon destruction.
    ~AdaptiveResourcePool() {
        std::lock_guard<std::mutex> lk(mutex_);
        for (size_t i = 0; i < resources_.size(); ++i) {
            if (!released_[i]) {
                if (params_.release_func) {
                    params_.release_func(resources_[i]);
                }
                resources_[i].reset();
            }
        }
        resources_.clear();
        busy_.clear();
        released_.clear();
        params_.logger("AdaptiveResourcePool destroyed.");
    }

    /// Acquires an available resource.
    /// Returns nullptr if no resources are currently available.
    T* acquire() {
        std::lock_guard<std::mutex> lk(mutex_);
        maybeRecover();

        for (size_t i = 0; i < resources_.size(); ++i) {
            if (!busy_[i].load() && !released_[i]) {
                // Check if we should release instead of using it
                if (params_.should_release && params_.should_release(activeCount())) {
                    maybeReleaseOne(i);
                    break;
                }
                busy_[i].store(true);
                return resources_[i].get();
            }
        }
        return nullptr;
    }

    /// Releases a previously acquired resource back into the pool.
    void release(T* res_ptr) {
        std::lock_guard<std::mutex> lk(mutex_);
        for (size_t i = 0; i < resources_.size(); ++i) {
            if (resources_[i].get() == res_ptr) {
                busy_[i].store(false);
                return;
            }
        }
        params_.logger("Tried to release unknown resource.");
    }

    /// Returns the number of idle (available) resources.
    size_t idleCount() const {
        std::lock_guard<std::mutex> lk(mutex_);
        size_t count = 0;
        for (size_t i = 0; i < resources_.size(); ++i) {
            if (!busy_[i].load() && !released_[i]) {
                ++count;
            }
        }
        return count;
    }

private:
    /// Counts the number of active (non-released) resources.
    size_t activeCount() const {
        return std::count(released_.begin(), released_.end(), false);
    }

    /// Attempts to restore released resources if allowed.
    void maybeRecover() {
        size_t active = activeCount();
        if (!params_.can_restore || !params_.can_restore(active))
            return;

        for (size_t i = 0; i < resources_.size(); ++i) {
            if (released_[i]) {
                auto restored = params_.restore_func(i);
                if (restored) {
                    resources_[i] = std::move(restored);
                    released_[i] = false;
                    busy_[i].store(false);
                    params_.logger("Restored resource[" + std::to_string(i) + "]");
                } else {
                    params_.logger("Failed to restore resource[" + std::to_string(i) + "]");
                }
            }
        }
    }

    /// Releases one idle resource starting from the given index.
    void maybeReleaseOne(size_t start_index) {
        size_t active = activeCount();
        for (size_t i = start_index; i < resources_.size(); ++i) {
            if (!busy_[i].load() && !released_[i]) {
                if (active <= 1)
                    break;
                busy_[i].store(true);
                params_.release_func(resources_[i]);
                resources_[i].reset();
                released_[i] = true;
                params_.logger("Released resource[" + std::to_string(i) + "]");
                break;
            }
        }
    }

private:
    Params params_; ///< Pool configuration parameters
    std::vector<std::unique_ptr<T>> resources_; ///< Managed resources
    std::vector<MovableAtomicBool> busy_; ///< Busy flags for each resource
    std::vector<bool> released_; ///< Release state flags
    mutable std::mutex mutex_; ///< Synchronization mutex
};
