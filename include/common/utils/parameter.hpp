#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>

#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#define GEN_PARAM(TYPE, NAME) \
    wust_vl::common::utils::Param<TYPE> NAME##_param { \
        #NAME \
    }
namespace wust_vl {
namespace common {
    namespace utils {

        template<typename T>
        class Param {
        public:
            using Callback = std::function<void(const T& old_v, const T& new_v)>;
            Param(const std::string& k) {
                key_ = k;
            }

            inline void load(const YAML::Node& node) {
                if (!node || !node[key_]) {
                    return;
                }

                T new_val = node[key_].as<T>();
                if (new_val != val_) {
                    T old = val_;
                    val_ = new_val;
                    if (on_change_) {
                        on_change_(old, new_val);
                    }
                }
            }

            inline const T& get() const {
                return val_;
            }
            inline void set(const T& v) {
                val_ = v;
            }

            inline void onChange(Callback cb) {
                on_change_ = std::move(cb);
            }

        private:
            T val_;
            Callback on_change_;
            std::string key_;
        };

        class ParamGroup {
        public:
            virtual ~ParamGroup() = default;

            virtual const char* key() const = 0;

            virtual void load(const YAML::Node& node) {
                loadSelf(node);
                loadChildren(node);
            }

        protected:
            virtual void loadSelf(const YAML::Node&) {}

            void registerChild(ParamGroup& child) {
                children_.emplace(child.key(), &child);
            }

        private:
            inline void loadChildren(const YAML::Node& node) {
                for (auto& [key, child]: children_) {
                    if (node[key]) {
                        child->load(node[key]);
                    }
                }
            }

            std::unordered_map<std::string, ParamGroup*> children_;
        };
        template<typename Derived>
        struct SimpleConfigBase: public wust_vl::common::utils::ParamGroup {
            using Ptr = std::shared_ptr<Derived>;

            static Ptr create() {
                return std::make_shared<Derived>();
            }

            bool first_load = false;

            const char* key() const override {
                return Derived::kKey;
            }

        protected:
            template<typename T, typename InitFn, typename UpdateFn = std::nullptr_t>
            inline void loadOnceOrUpdate(
                const YAML::Node& node,
                T& value,
                InitFn&& init_fn,
                UpdateFn&& update_fn = nullptr
            ) {
                if (!first_load) {
                    init_fn(node, value);
                    first_load = true;
                } else if constexpr (!std::is_same_v<UpdateFn, std::nullptr_t>) {
                    update_fn(node, value);
                }
            }
        };
        class Parameter {
        public:
            using Ptr = std::shared_ptr<Parameter>;
            static Ptr create() {
                return std::make_shared<Parameter>();
            }
            inline YAML::Node getConfig() const {
                return YAML::Clone(cfg_);
            }
            inline void registerGroup(ParamGroup& group) {
                groups_[group.key()].push_back(&group);
            }
            inline bool reloadFromOldPath() {
                cfg_ = YAML::LoadFile(path_);
                reload();
                return true;
            }
            inline bool loadFromFile(const std::string& path) {
                path_ = path;
                cfg_ = YAML::LoadFile(path_);
                reload();
                return true;
            }

            inline bool reload() {
                if (!cfg_) {
                    throw std::runtime_error("Parameter: YAML not loaded");
                }

                for (auto& [key, vec]: groups_) {
                    if (!cfg_[key])
                        continue;

                    const YAML::Node& node = cfg_[key];
                    for (auto* group: vec) {
                        group->load(node);
                    }
                }
                return true;
            }

        private:
            std::string path_;
            YAML::Node cfg_;
            std::unordered_map<std::string, std::vector<ParamGroup*>> groups_;
        };
        class ParameterManager {
        public:
            static inline ParameterManager& instance() {
                static ParameterManager inst;
                return inst;
            }
            inline void registerParameter(Parameter& parameter) {
                parameters_.push_back(&parameter);
            }
            inline void allReloadFromOldPath() {
                for (auto* parameter: parameters_) {
                    parameter->reloadFromOldPath();
                }
            }

        private:
            std::mutex mtx_;
            std::vector<Parameter*> parameters_;
        };
    } // namespace utils
} // namespace common
} // namespace wust_vl
