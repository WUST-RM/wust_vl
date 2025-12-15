#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unordered_map>
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
        *var = default_value;

        bindings_.emplace_back([=, this]() mutable {
            YAML::Node node = traverse(keys);
            if (node && node.IsDefined()) {
                try {
                    T value = node.as<T>();
                    if (value != *last_value) {
                        *var = value;
                        *last_value = value;
                        logUpdate(keys, value);
                    }
                } catch (const std::exception& e) {
                    logTypeError(keys, e.what(), *last_value);
                }
            } else {
                if (*last_value != default_value) {
                    *var = default_value;
                    *last_value = default_value;
                    logMissing(keys, default_value);
                }
            }
        });

        bindings_.back()();
    }

    template<typename T>
    void bind(const std::vector<std::string>& keys, T* var) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto last_value = std::make_shared<T>();

        bindings_.emplace_back([=, this]() mutable {
            YAML::Node node = traverse(keys);
            if (!node || !node.IsDefined()) {
                throw std::runtime_error("[ConfigBinder] Missing required key");
            }
            try {
                T value = node.as<T>();
                if (!last_value || value != *last_value) {
                    *var = value;
                    *last_value = value;
                    logUpdate(keys, value);
                }
            } catch (const std::exception& e) {
                throw std::runtime_error(
                    "[ConfigBinder] Type error on required key: " + std::string(e.what())
                );
            }
        });

        bindings_.back()();
    }

    void reload() {
        std::lock_guard<std::mutex> lock(mutex_);
        load(config_path_);
        for (auto& fn: bindings_) {
            fn();
        }
    }

    void reload(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        load(path);
        for (auto& fn: bindings_) {
            fn();
        }
    }

private:
    void load(const std::string& path) {
        try {
            config_ = YAML::LoadFile(path);
        } catch (const std::exception& e) {
            throw std::runtime_error("[ConfigBinder] Failed to load: " + std::string(e.what()));
        }
    }

    YAML::Node traverse(const std::vector<std::string>& keys) const {
        YAML::Node node = YAML::Clone(config_);
        for (const auto& key: keys) {
            if (!node || !node[key]) {
                return YAML::Node();
            }
            node = node[key];
        }
        return node;
    }

    template<typename T>
    static void logUpdate(const std::vector<std::string>& keys, const T& value) {
        std::cout << "[ConfigBinder] Key=";
        for (auto& k: keys)
            std::cout << "\"" << k << "\" ";
        std::cout << "Updated Value=" << value << " (type=" << typeid(T).name() << ")\n";
    }

    template<typename T>
    static void logMissing(const std::vector<std::string>& keys, const T& fallback) {
        std::cerr << "[ConfigBinder] Missing key: ";
        for (auto& k: keys)
            std::cerr << k << " ";
        std::cerr << ", fallback=" << fallback << "\n";
    }

    static void
    logTypeError(const std::vector<std::string>& keys, const std::string& err, const auto& keep) {
        std::cerr << "[ConfigBinder] Type error on key: ";
        for (auto& k: keys)
            std::cerr << k << " ";
        std::cerr << " -> " << err << ", keeping old=" << keep << "\n";
    }

private:
    YAML::Node config_;
    std::vector<std::function<void()>> bindings_;
    std::mutex mutex_;
    std::string config_path_;
};

class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager inst;
        return inst;
    }

    void registerConfig(const std::string& name, std::shared_ptr<ConfigBinder> binder) {
        if (configs_.count(name)) {
            throw std::runtime_error("Config '" + name + "' already registered.");
        }
        configs_[name] = std::move(binder);
    }

    std::shared_ptr<ConfigBinder> get(const std::string& name) {
        auto it = configs_.find(name);
        if (it == configs_.end()) {
            throw std::runtime_error("Config '" + name + "' not found.");
        }
        return it->second;
    }

    bool has(const std::string& name) const {
        return configs_.count(name) != 0;
    }

    void remove(const std::string& name) {
        configs_.erase(name);
    }

    void reload() {
        for (auto& [_, cfg]: configs_) {
            cfg->reload();
        }
    }

private:
    ConfigManager() = default;
    std::unordered_map<std::string, std::shared_ptr<ConfigBinder>> configs_;
};

template<typename T>
inline void bindConfig(
    std::shared_ptr<ConfigBinder> binder,
    const std::vector<std::string>& keys,
    T* var,
    const T& default_value
) {
    binder->bind(keys, var, default_value);
}

template<typename T>
inline void
bindConfig(std::shared_ptr<ConfigBinder> binder, const std::vector<std::string>& keys, T* var) {
    binder->bind(keys, var);
}

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
