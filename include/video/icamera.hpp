#pragma once
#include "image.hpp"
#include <yaml-cpp/node/node.h>
namespace wust_vl {
namespace video {

    class ICameraDevice {
    public:
        virtual ~ICameraDevice() = default;

        virtual void start() = 0;
        virtual void stop() = 0;
        virtual bool read() = 0;
        virtual ImageFrame readImage() = 0;
        using FrameCallback = std::function<void(ImageFrame&)>;
        virtual void setFrameCallback(FrameCallback cb) = 0;

        virtual bool loadConfig(const YAML::Node& cfg) = 0;
    };

} // namespace video
} // namespace wust_vl
