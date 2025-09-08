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
namespace wust_vl_utils {
class ConfigBinder {
public:
    /// 从文件初始化
    explicit ConfigBinder(const std::string& path): config_path_(path) {
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
    void reload() {
        std::lock_guard<std::mutex> lock(mutex_);
        load(config_path_);
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
    std::string config_path_;
};

/// bindConfig 辅助函数（有默认值）
template<typename T>
inline void bindConfig(
    std::shared_ptr<ConfigBinder> binder,
    const std::vector<std::string>& keys,
    T* var,
    const T& default_value
) {
    try {
        binder->bind(keys, var, default_value);
    } catch (YAML::Exception& e) {
        std::cerr << "[ConfigBinder]"
                  << "bind error" << e.what() << std::endl;
    }
}

/// bindConfig 辅助函数（无默认值）
template<typename T>
inline void
bindConfig(std::shared_ptr<ConfigBinder> binder, const std::vector<std::string>& keys, T* var) {
    try {
        binder->bind(keys, var);
    } catch (YAML::Exception& e) {
        std::cerr << "[ConfigBinder]"
                  << "bind error" << e.what() << std::endl;
    }
}
class ConfigManager {
public:
    // 获取全局单例
    static ConfigManager& instance() {
        static ConfigManager inst;
        return inst;
    }

    // 注册一个 ConfigBinder
    void registerConfig(const std::string& name, std::shared_ptr<ConfigBinder> binder) {
        if (configs_.count(name)) {
            throw std::runtime_error("Config with name '" + name + "' already registered.");
        }
        configs_[name] = std::move(binder);
    }

    // 按名字获取 ConfigBinder
    std::shared_ptr<ConfigBinder> get(const std::string& name) {
        auto it = configs_.find(name);
        if (it == configs_.end()) {
            throw std::runtime_error("Config with name '" + name + "' not found.");
        }
        return it->second;
    }

    // 判断是否存在
    bool has(const std::string& name) const {
        return configs_.count(name) > 0;
    }

    // 移除一个配置
    void remove(const std::string& name) {
        configs_.erase(name);
    }
    void reload() {
        for (auto& cfg: configs_) {
            cfg.second->reload();
        }
    }

private:
    ConfigManager() = default;
    std::unordered_map<std::string, std::shared_ptr<ConfigBinder>> configs_;
};
template<typename T>
inline void
bindConfigByManager(const std::string& name, const std::vector<std::string>& keys, T* var) {
    bindConfig(ConfigManager::instance().get(name), keys, var);
}
template<typename T>
inline void bindConfigByManager(
    const std::string& name,
    const std::vector<std::string>& keys,
    T* var,
    const T& default_value
) {
    bindConfig(ConfigManager::instance().get(name), keys, var, default_value);
}

} // namespace wust_vl_utils
