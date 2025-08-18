#pragma once

#include <any>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stringanything {

// ========== 异常 ==========
struct KeyNotFound: public std::runtime_error {
    using std::runtime_error::runtime_error;
};
struct BadType: public std::runtime_error {
    using std::runtime_error::runtime_error;
};
struct AlreadyDefined: public std::runtime_error {
    using std::runtime_error::runtime_error;
};
struct TimeoutError: public std::runtime_error {
    using std::runtime_error::runtime_error;
};

// ========== 静态 Holder（以 shared_ptr 形式存放可拷贝值） ==========
class IStaticHolder {
public:
    virtual ~IStaticHolder() = default;
    virtual std::type_index type() const = 0;
};

template<typename T>
class StaticHolder: public IStaticHolder {
public:
    using Decayed = std::decay_t<T>;
    explicit StaticHolder(Decayed v): value_(std::move(v)) {}
    std::type_index type() const override {
        return typeid(Decayed);
    }

    Decayed get_copy() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return value_;
    }
    void set(Decayed v) {
        std::lock_guard<std::mutex> lk(mtx_);
        value_ = std::move(v);
    }
    template<typename F>
    void update(F&& f) {
        std::lock_guard<std::mutex> lk(mtx_);
        f(value_); // f: void(Decayed&)
    }

private:
    mutable std::mutex mtx_;
    Decayed value_;
};

// ========== 管理器：按字符串存储 value 或 shared_ptr<T> ==========
class Manager {
public:
    using sptr = std::shared_ptr<Manager>;
    static sptr create() {
        return std::make_shared<Manager>();
    }

    Manager() = default;

    enum class Kind { Value, Ptr };

    // ---- set_value: 存放一个值（通过 get_value 返回拷贝） ----
    // overwrite: 是否允许覆盖已存在条目（若类型不匹配抛 BadType）
    template<typename T>
    void set_value(const std::string& name, T value, bool overwrite = true) {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        std::unique_lock lk(map_mtx_);
        auto it = map_.find(name);
        if (it != map_.end()) {
            if (it->second.kind != Kind::Value)
                throw BadType("existing entry is Ptr, not Value: " + name);
            if (it->second.type != ty)
                throw BadType("type mismatch for value entry: " + name);
            if (!overwrite)
                throw AlreadyDefined("value exists: " + name);

            auto hold = std::any_cast<std::shared_ptr<StaticHolder<Decayed>>>(it->second.storage);
            hold->set(std::move(value));
            return;
        }
        auto holder = std::make_shared<StaticHolder<Decayed>>(std::move(value));
        Entry e { Kind::Value, ty, std::any(holder) };
        map_.emplace(name, std::move(e));
    }

    // ---- get_value: 返回拷贝（抛异常版本）----
    template<typename T>
    [[nodiscard]] std::decay_t<T> get_value(const std::string& name) const {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        std::shared_lock lk(map_mtx_);
        auto it = map_.find(name);
        if (it == map_.end())
            throw KeyNotFound("value not found: " + name);
        const Entry& e = it->second;
        if (e.kind != Kind::Value)
            throw BadType("entry is Ptr, not Value: " + name);
        if (e.type != ty)
            throw BadType("value type mismatch: " + name);

        auto holder = std::any_cast<std::shared_ptr<StaticHolder<Decayed>>>(e.storage);
        return holder->get_copy();
    }

    // ---- try_get_value: 返回 std::optional（不抛异常）----
    template<typename T>
    [[nodiscard]] std::optional<std::decay_t<T>> try_get_value(const std::string& name
    ) const noexcept {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        std::shared_lock lk(map_mtx_);
        auto it = map_.find(name);
        if (it == map_.end())
            return std::nullopt;
        const Entry& e = it->second;
        if (e.kind != Kind::Value || e.type != ty)
            return std::nullopt;

        try {
            auto holder = std::any_cast<std::shared_ptr<StaticHolder<Decayed>>>(e.storage);
            return holder ? std::optional<Decayed>(holder->get_copy()) : std::nullopt;
        } catch (...) {
            return std::nullopt;
        }
    }

    // ---- 原子更新已存在的 value（不改变类型）----
    template<typename T, typename F>
    bool try_update_value(const std::string& name, F&& updater) {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        std::shared_ptr<StaticHolder<Decayed>> holder;
        {
            std::shared_lock rlk(map_mtx_);
            auto it = map_.find(name);
            if (it == map_.end())
                return false;
            const Entry& e = it->second;
            if (e.kind != Kind::Value || e.type != ty)
                return false;
            try {
                holder = std::any_cast<std::shared_ptr<StaticHolder<Decayed>>>(e.storage);
            } catch (...) {
                return false;
            }
        }
        if (!holder)
            return false;
        holder->update(std::forward<F>(updater)); // 内部自带锁
        return true;
    }

    // ---- set_ptr: 存放 shared_ptr<T> ----
    template<typename T>
    void set_ptr(const std::string& name, std::shared_ptr<T> p, bool overwrite = true) {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);
        if (!p)
            throw BadType("null shared_ptr for: " + name);

        std::unique_lock lk(map_mtx_);
        auto it = map_.find(name);
        if (it != map_.end()) {
            if (it->second.kind != Kind::Ptr)
                throw BadType("existing entry is Value: " + name);
            if (it->second.type != ty)
                throw BadType("type mismatch: " + name);
            if (!overwrite)
                throw AlreadyDefined("ptr exists: " + name);
            it->second.storage = std::any(std::move(p));
            return;
        }
        Entry e { Kind::Ptr, ty, std::any(std::move(p)) };
        map_.emplace(name, std::move(e));
    }

    template<typename T>
    [[nodiscard]] std::shared_ptr<std::decay_t<T>> get_ptr(const std::string& name) const {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        std::shared_lock lk(map_mtx_);
        auto it = map_.find(name);
        if (it == map_.end())
            throw KeyNotFound("ptr not found: " + name);
        const Entry& e = it->second;
        if (e.kind != Kind::Ptr)
            throw BadType("entry is Value: " + name);
        if (e.type != ty)
            throw BadType("type mismatch: " + name);

        auto p = std::any_cast<std::shared_ptr<Decayed>>(e.storage);
        if (!p)
            throw BadType("stored ptr is null: " + name);
        return p;
    }

    template<typename T>
    [[nodiscard]] std::optional<std::shared_ptr<std::decay_t<T>>>
    try_get_ptr(const std::string& name) const noexcept {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        std::shared_lock lk(map_mtx_);
        auto it = map_.find(name);
        if (it == map_.end())
            return std::nullopt;
        const Entry& e = it->second;
        if (e.kind != Kind::Ptr || e.type != ty)
            return std::nullopt;

        try {
            auto p = std::any_cast<std::shared_ptr<Decayed>>(e.storage);
            if (!p)
                return std::nullopt;
            return p;
        } catch (...) {
            return std::nullopt;
        }
    }

    // ---- create_ptr: factory 创建并返回 shared_ptr<T>，若已存在按 overwrite 决策 ----
    template<typename T>
    std::shared_ptr<std::decay_t<T>> create_ptr(
        const std::string& name,
        std::function<std::shared_ptr<std::decay_t<T>>()> factory,
        bool overwrite = false
    ) {
        using Decayed = std::decay_t<T>;
        const std::type_index ty = typeid(Decayed);

        {
            std::shared_lock rlk(map_mtx_);
            auto it = map_.find(name);
            if (it != map_.end() && !overwrite) {
                if (it->second.kind != Kind::Ptr)
                    throw BadType("existing entry kind mismatch: " + name);
                if (it->second.type != ty)
                    throw BadType("existing entry type mismatch: " + name);
                return std::any_cast<std::shared_ptr<Decayed>>(it->second.storage);
            }
        }
        // 需要创建或覆盖
        auto p = factory();
        if (!p)
            throw BadType("factory returned null for: " + name);

        std::unique_lock wlk(map_mtx_);
        auto it = map_.find(name);
        if (it != map_.end()) {
            if (it->second.kind != Kind::Ptr)
                throw BadType("existing entry kind mismatch: " + name);
            if (it->second.type != ty)
                throw BadType("existing entry type mismatch: " + name);
            it->second.storage = std::any(p);
        } else {
            Entry e { Kind::Ptr, ty, std::any(p) };
            map_.emplace(name, std::move(e));
        }
        return p;
    }

    // ---- exists / erase / names / size / clear ----
    [[nodiscard]] bool exists(const std::string& name) const {
        std::shared_lock lk(map_mtx_);
        return map_.find(name) != map_.end();
    }

    void erase(const std::string& name) {
        std::unique_lock lk(map_mtx_);
        map_.erase(name);
    }

    [[nodiscard]] std::vector<std::string> names() const {
        std::shared_lock lk(map_mtx_);
        std::vector<std::string> out;
        out.reserve(map_.size());
        for (const auto& kv: map_)
            out.push_back(kv.first);
        return out;
    }

    [[nodiscard]] std::size_t size() const {
        std::shared_lock lk(map_mtx_);
        return map_.size();
    }

    void clear() {
        std::unique_lock lk(map_mtx_);
        map_.clear();
    }

private:
    struct Entry {
        Kind kind = Kind::Value;
        std::type_index type = std::type_index(typeid(void));
        std::any storage;

        Entry() noexcept = default;
        Entry(Kind k, std::type_index t, std::any s): kind(k), type(t), storage(std::move(s)) {}
    };

    mutable std::shared_mutex map_mtx_;
    std::unordered_map<std::string, Entry> map_;
};

} // namespace stringanything
