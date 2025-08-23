#include "video/camera.hpp"
#include "common/utils/logger.hpp"
namespace wust_vl_video {
struct Camera::Impl {
    bool init(const YAML::Node& config) {
        config_ = config;
        type_ = string2CameraType(config["type"].as<std::string>());
        switch (type_) {
            case CameraType::HIK: {
                hik_camera_ = std::make_unique<HikCamera>();
                std::string target_sn = config["hik_camera"]["target_sn"].as<std::string>();
                if (!hik_camera_->initializeCamera(target_sn)) {
                    WUST_ERROR("camera") << "Camera initialization failed.";
                    return false;
                }

                hik_camera_->setParameters(
                    config["hik_camera"]["acquisition_frame_rate"].as<int>(),
                    config["hik_camera"]["exposure_time"].as<int>(),
                    config["hik_camera"]["gain"].as<double>(),
                    config["hik_camera"]["gamma"].as<double>(),
                    config["hik_camera"]["adc_bit_depth"].as<std::string>(),
                    config["hik_camera"]["pixel_format"].as<std::string>(),
                    config["hik_camera"]["acquisition_frame_rate_enable"].as<bool>(),
                    config["hik_camera"]["reverse_x"].as<bool>(false),
                    config["hik_camera"]["reverse_y"].as<bool>(false)
                );
                return true;
            }

            case CameraType::VIDEO_PLAYER: {
                std::string video_play_path = config["video_player"]["path"].as<std::string>("");
                int video_play_fps = config["video_player"]["fps"].as<int>(30);
                int start_frame = config["video_player"]["start_frame"].as<int>(0);
                bool loop = config["video_player"]["loop"].as<bool>(false);
                video_player_ = std::make_unique<VideoPlayer>(
                    video_play_path,
                    video_play_fps,
                    start_frame,
                    loop
                );
                return true;
            }
        }
        return true;
    }
    void read() {
        if (hik_camera_) {
            hik_camera_->read();
        }
        if (video_player_) {
            video_player_->read();
        }
    }
    ImageFrame readImage() {
        if (hik_camera_) {
            return hik_camera_->readImage();
        }
    }
    void setFrameCallback(std::function<void(ImageFrame&)> cb) {
        if (hik_camera_) {
            std::function<void(ImageFrame&)> real_cb = [cb](ImageFrame& frame) {
                frame.src_img = convertToMat(frame);
                cb(frame);
            };
            hik_camera_->setFrameCallback(real_cb);
        }
        if (video_player_) {
            video_player_->setCallback(cb);
        }
    }
    void start() {
        if (hik_camera_) {
            bool if_recorder = config_["hik_camera"]["recorder"].as<bool>(false);
            hik_camera_->startCamera(if_recorder);
        }
        if (video_player_) {
            video_player_->start();
        }
    }
    void stop() {
        if (video_player_) {
            video_player_->stop();
            video_player_.reset();
        }
        if (hik_camera_) {
            hik_camera_->stopCamera();
            hik_camera_.reset();
        }
    }
    CameraType type_;
    std::unique_ptr<HikCamera> hik_camera_;
    std::unique_ptr<VideoPlayer> video_player_;
    YAML::Node config_;
};
Camera::Camera() {
    _impl = std::make_unique<Impl>();
}
Camera::~Camera() {
    _impl.reset();
}
bool Camera::init(const YAML::Node& config) {
    return _impl->init(config);
}
void Camera::start() {
    _impl->start();
}
void Camera::stop() {
    _impl->stop();
}
ImageFrame Camera::readImage() {
    return _impl->readImage();
}
void Camera::setFrameCallback(std::function<void(ImageFrame&)> cb) {
    _impl->setFrameCallback(cb);
}
} // namespace wust_vl_video
