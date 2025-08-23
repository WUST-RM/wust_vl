#include <yaml-cpp/yaml.h>
#include <functional>
#include <vector>
#include <string>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <typeinfo>

class ConfigBinder {
public:
    /// 从文件初始化
    explicit ConfigBinder(const std::string& path) { load(path); }
    /// 从 YAML::Node 初始化
    explicit ConfigBinder(const YAML::Node& node) { load(node); }

    /// 有默认值的 bind
    template<typename T>
    void bind(const std::string& key, T* var, const T& default_value) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto last_value = std::make_shared<T>(default_value);

        bindings_.push_back([=, this]() mutable {
            if (config_[key]) {
                try {
                    T value = config_[key].as<T>();
                    if (value != *last_value) {
                        *var = value;
                        *last_value = value;
                        std::cout << "[ConfigBinder] Key=\"" << key
                                  << "\" Updated Value=" << value
                                  << " (type=" << typeid(T).name() << ")\n";
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[ConfigBinder] Type error on key: " 
                              << key << " -> " << e.what()
                              << ", keeping old=" << *last_value << "\n";
                }
            } else {
                if (*last_value != default_value) {
                    *var = default_value;
                    *last_value = default_value;
                    std::cerr << "[ConfigBinder] Missing key: " << key
                              << ", fallback=" << default_value << "\n";
                }
            }
        });
        bindings_.back()();
    }

    /// 无默认值的 bind（必须存在）
    template<typename T>
    void bind(const std::string& key, T* var) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto last_value = std::make_shared<T>();

        bindings_.push_back([=, this]() mutable {
            if (!config_[key]) {
                throw std::runtime_error("[ConfigBinder] Missing required key: " + key);
            }
            try {
                T value = config_[key].as<T>();
                if (value != *last_value) {
                    *var = value;
                    *last_value = value;
                    std::cout << "[ConfigBinder] Key=\"" << key
                              << "\" Updated Value=" << value
                              << " (type=" << typeid(T).name() << ")\n";
                }
            } catch (const std::exception& e) {
                throw std::runtime_error("[ConfigBinder] Type error on required key: " 
                                         + key + " -> " + e.what());
            }
        });
        bindings_.back()();
    }

    /// reload（文件路径）
    void reload(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        load(path);
        //std::cout << "[ConfigBinder] Reloading from file: " << path << "\n";
        for (auto& fn : bindings_) fn();
    }

    /// reload（YAML::Node）
    void reload(const YAML::Node& node) {
        std::lock_guard<std::mutex> lock(mutex_);
        load(node);
        //std::cout << "[ConfigBinder] Reloading from YAML::Node\n";
        for (auto& fn : bindings_) fn();
    }

private:
    void load(const std::string& path) {
        try {
            config_ = YAML::LoadFile(path);
            //std::cout << "[ConfigBinder] Loaded file: " << path << "\n";
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("[ConfigBinder] Failed to load: ") + e.what());
        }
    }

    void load(const YAML::Node& node) {
        config_ = node;
        std::cout << "[ConfigBinder] Loaded from Node\n";
    }

    YAML::Node config_;
    std::vector<std::function<void()>> bindings_;
    std::mutex mutex_;
};


template<typename T>
inline void bindConfig(std::shared_ptr<ConfigBinder> binder,
                       const std::string& key,
                       T* var,
                       const T& default_value,
                       const std::string& fallback_path = "config.yaml") {
    if (binder) {
        binder->bind(key, var, default_value);
    } else {
        try {
            YAML::Node config = YAML::LoadFile(fallback_path);
            if (config[key]) {
                *var = config[key].as<T>();
                std::cout << "[bindConfig] (no binder) Key=\"" << key 
                          << "\" Value=" << *var << "\n";
            } else {
                *var = default_value;
                std::cerr << "[bindConfig] (no binder) Missing key: " << key
                          << ", fallback=" << default_value << "\n";
            }
        } catch (const std::exception& e) {
            *var = default_value;
            std::cerr << "[bindConfig] (no binder) Load failed: " << e.what()
                      << ", fallback=" << default_value << "\n";
        }
    }
}

/// 无默认值版本
template<typename T>
inline void bindConfig(std::shared_ptr<ConfigBinder> binder,
                       const std::string& key,
                       T* var,
                       const std::string& fallback_path = "config.yaml") {
    if (binder) {
        binder->bind(key, var);
    } else {
        try {
            YAML::Node config = YAML::LoadFile(fallback_path);
            if (!config[key]) {
                throw std::runtime_error("[bindConfig] (no binder) Missing required key: " + key);
            }
            *var = config[key].as<T>();
            std::cout << "[bindConfig] (no binder) Key=\"" << key 
                      << "\" Value=" << *var << "\n";
        } catch (const std::exception& e) {
            throw std::runtime_error("[bindConfig] (no binder) Load failed: " + std::string(e.what()));
        }
    }
}
