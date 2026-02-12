#pragma once
#include <cuda_runtime.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
namespace cuda_cvt {
enum class BayerPattern { BGGR = 0, GBRG = 1, GRBG = 2, RGGB = 3 };

enum class OutputOrder { BGR = 0, RGB = 1 };
enum class InterpMode { EA = 0, BILINEAR = 1 };

struct CvBayerDesc {
    BayerPattern pattern;
    OutputOrder order;
};

class CudaBayer {
public:
    CudaBayer();
    ~CudaBayer();

    // cv::Mat -> cv::Mat（回传 CPU）
    void process(const cv::Mat& bayer, cv::Mat& rgb, int cv_bayer_code);

private:
    void allocIfNeeded(int width, int height);
    void release();

private:
    uint8_t* d_bayer_ = nullptr;
    uint8_t* d_target_ = nullptr;

    int width_ = 0;
    int height_ = 0;

    cudaStream_t stream_ = nullptr;
    size_t bayer_capacity_ = 0;
    size_t rgb_capacity_ = 0;
};
} // namespace cuda_cvt