#pragma once
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>
#include <yaml-cpp/yaml.h>

class ConfigBinder {
public:
    /// 从文件初始化
    explicit ConfigBinder(const std::string& path) {
        load(path);
    }

    /// 有默认值的 bind
    template<typename T>
    void bind(const std::vector<std::string>& keys, T* var, const T& default_value) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto last_value = std::make_shared<T>(default_value);

        bindings_.push_back([=, this]() mutable {
            YAML::Node node = traverse(keys);
            if (node && node.IsDefined()) {
                try {
                    T value = node.as<T>();
                    if (value != *last_value) {
                        *var = value;
                        *last_value = value;
                        std::cout << "[ConfigBinder] Key=";
                        for (auto& k: keys)
                            std::cout << "\"" << k << "\" ";
                        std::cout << "Updated Value=" << value << " (type=" << typeid(T).name()
                                  << ")\n";
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[ConfigBinder] Type error on key: ";
                    for (auto& k: keys)
                        std::cerr << k << " ";
                    std::cerr << " -> " << e.what() << ", keeping old=" << *last_value << "\n";
                }
            } else {
                if (*last_value != default_value) {
                    *var = default_value;
                    *last_value = default_value;
                    std::cerr << "[ConfigBinder] Missing key: ";
                    for (auto& k: keys)
                        std::cerr << k << " ";
                    std::cerr << ", fallback=" << default_value << "\n";
                }
            }
        });

        // 初次执行一次以设置初始值
        bindings_.back()();
    }

    /// 无默认值的 bind（必须存在）
    template<typename T>
    void bind(const std::vector<std::string>& keys, T* var) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto last_value = std::make_shared<T>();

        bindings_.push_back([=, this]() mutable {
            YAML::Node node = traverse(keys);
            if (!node || !node.IsDefined()) {
                throw std::runtime_error("[ConfigBinder] Missing required key");
            }
            try {
                T value = node.as<T>();
                if (value != *last_value) {
                    *var = value;
                    *last_value = value;
                    std::cout << "[ConfigBinder] Key=";
                    for (auto& k: keys)
                        std::cout << "\"" << k << "\" ";
                    std::cout << "Updated Value=" << value << " (type=" << typeid(T).name()
                              << ")\n";
                }
            } catch (const std::exception& e) {
                throw std::runtime_error(
                    "[ConfigBinder] Type error on required key -> " + std::string(e.what())
                );
            }
        });

        // 初次执行一次以设置初始值或抛出异常
        bindings_.back()();
    }

    /// reload（文件路径）
    void reload(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        load(path);
        for (auto& fn: bindings_)
            fn();
    }

private:
    void load(const std::string& path) {
        try {
            config_ = YAML::LoadFile(path);
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("[ConfigBinder] Failed to load: ") + e.what());
        }
    }

    YAML::Node traverse(const std::vector<std::string>& keys) const {
        YAML::Node node = YAML::Clone(config_); // 深拷贝整个 config_
        for (const auto& key: keys) {
            if (!node || !node[key])
                return YAML::Node(); // 不存在就返回空节点
            node = node[key];
        }
        return node;
    }

    YAML::Node config_;
    std::vector<std::function<void()>> bindings_;
    std::mutex mutex_;
};

/// bindConfig 辅助函数（有默认值）
template<typename T>
inline void bindConfig(
    std::shared_ptr<ConfigBinder> binder,
    const std::vector<std::string>& keys,
    T* var,
    const T& default_value,
    const std::string& fallback_path = "config.yaml"
) {
    if (binder) {
        binder->bind(keys, var, default_value);
    } else {
        try {
            YAML::Node config = YAML::LoadFile(fallback_path);
            YAML::Node node = config;
            for (auto& k: keys)
                node = node[k];
            if (node && node.IsDefined()) {
                *var = node.as<T>();
            } else {
                *var = default_value;
            }
        } catch (...) {
            *var = default_value;
        }
    }
}

/// bindConfig 辅助函数（无默认值）
template<typename T>
inline void bindConfig(
    std::shared_ptr<ConfigBinder> binder,
    const std::vector<std::string>& keys,
    T* var,
    const std::string& fallback_path = "config.yaml"
) {
    if (binder) {
        binder->bind(keys, var);
    } else {
        YAML::Node config = YAML::LoadFile(fallback_path);
        YAML::Node node = config;
        for (auto& k: keys)
            node = node[k];
        if (!node || !node.IsDefined()) {
            throw std::runtime_error("[bindConfig] Missing required key");
        }
        *var = node.as<T>();
    }
}
