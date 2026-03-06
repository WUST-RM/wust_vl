#include "cvtcolor.hpp"
#include <iostream>
#include <npp.h>
#include <nppcore.h>
#include <nppdefs.h>
#include <nppi_color_conversion.h>

namespace cuda_cvt {
constexpr int BLOCK_W = 16;
constexpr int BLOCK_H = 16;

template<BayerPattern P>
__device__ __forceinline__ constexpr int bayerColor(int x, int y) {
    int idx = ((y & 1) << 1) | (x & 1);

    if constexpr (P == BayerPattern::RGGB)
        return (idx == 0) ? 0 : (idx == 3 ? 2 : 1);
    else if constexpr (P == BayerPattern::BGGR)
        return (idx == 0) ? 2 : (idx == 3 ? 0 : 1);
    else if constexpr (P == BayerPattern::GBRG)
        return (idx == 1) ? 2 : (idx == 2 ? 0 : 1);
    else // GRBG
        return (idx == 1) ? 0 : (idx == 2 ? 2 : 1);
}
template<OutputOrder O>
__device__ __forceinline__ void storeRGB(uint8_t* out, int R, int G, int B) {
    if constexpr (O == OutputOrder::RGB) {
        out[0] = (uint8_t)R;
        out[1] = (uint8_t)G;
        out[2] = (uint8_t)B;
    } else {
        out[0] = (uint8_t)B;
        out[1] = (uint8_t)G;
        out[2] = (uint8_t)R;
    }
}
__device__ __forceinline__ uint8_t load_clamped_dev(
    const uint8_t* __restrict__ bayer,
    int x,
    int y,
    int width,
    int height,
    int pitch
) {
    if (x < 0)
        x = 0;
    if (x >= width)
        x = width - 1;
    if (y < 0)
        y = 0;
    if (y >= height)
        y = height - 1;
    return bayer[y * pitch + x];
}

// --- shared-memory optimized kernel ---
template<BayerPattern P, OutputOrder O, InterpMode M>
__global__ void bayer_kernel_shared(
    const uint8_t* __restrict__ bayer,
    uint8_t* __restrict__ out,
    int width,
    int height,
    int pitch // pitch in bytes == width for linear layout
) {
    const int tx = threadIdx.x;
    const int ty = threadIdx.y;
    const int bx = blockDim.x;
    const int by = blockDim.y;

    const int gx = blockIdx.x * bx + tx;
    const int gy = blockIdx.y * by + ty;

    // shared tile with 1-pixel halo
    extern __shared__ uint8_t s[];
    const int sstride = bx + 2;
    const int sx = tx + 1;
    const int sy = ty + 1;

    // load center (clamped)
    uint8_t center_val = 0;
    if (gx < width && gy < height) {
        center_val = __ldg(&bayer[gy * pitch + gx]);
    } else {
        center_val = load_clamped_dev(bayer, gx, gy, width, height, pitch);
    }
    s[sy * sstride + sx] = center_val;

    // halo loads: left/right/top/bottom corners
    // left
    if (tx == 0) {
        s[sy * sstride + (sx - 1)] = load_clamped_dev(bayer, gx - 1, gy, width, height, pitch);
    }
    // right
    if (tx == bx - 1) {
        s[sy * sstride + (sx + 1)] = load_clamped_dev(bayer, gx + 1, gy, width, height, pitch);
    }
    // top
    if (ty == 0) {
        s[(sy - 1) * sstride + sx] = load_clamped_dev(bayer, gx, gy - 1, width, height, pitch);
    }
    // bottom
    if (ty == by - 1) {
        s[(sy + 1) * sstride + sx] = load_clamped_dev(bayer, gx, gy + 1, width, height, pitch);
    }
    // corners
    if (tx == 0 && ty == 0) {
        s[(sy - 1) * sstride + (sx - 1)] =
            load_clamped_dev(bayer, gx - 1, gy - 1, width, height, pitch);
    }
    if (tx == bx - 1 && ty == 0) {
        s[(sy - 1) * sstride + (sx + 1)] =
            load_clamped_dev(bayer, gx + 1, gy - 1, width, height, pitch);
    }
    if (tx == 0 && ty == by - 1) {
        s[(sy + 1) * sstride + (sx - 1)] =
            load_clamped_dev(bayer, gx - 1, gy + 1, width, height, pitch);
    }
    if (tx == bx - 1 && ty == by - 1) {
        s[(sy + 1) * sstride + (sx + 1)] =
            load_clamped_dev(bayer, gx + 1, gy + 1, width, height, pitch);
    }

    __syncthreads();

    // guard border pixels (we require valid 1-pixel neighborhood)
    if (gx <= 0 || gy <= 0 || gx >= width - 1 || gy >= height - 1) {
        return;
    }

    // load neighbors from shared memory
    const int c = s[sy * sstride + sx];
    const int l = s[sy * sstride + sx - 1];
    const int r = s[sy * sstride + sx + 1];
    const int u = s[(sy - 1) * sstride + sx];
    const int d = s[(sy + 1) * sstride + sx];
    const int ul = s[(sy - 1) * sstride + sx - 1];
    const int ur = s[(sy - 1) * sstride + sx + 1];
    const int dl = s[(sy + 1) * sstride + sx - 1];
    const int dr = s[(sy + 1) * sstride + sx + 1];

    int R = 0, G = 0, B = 0;
    int color = bayerColor<P>(gx, gy);

    constexpr bool G_on_R_row_pattern = (P == BayerPattern::RGGB || P == BayerPattern::GRBG);
    bool g_on_r = G_on_R_row_pattern ? ((gy & 1) == 0) : ((gy & 1) == 1);

    if constexpr (M == InterpMode::EA) {
        if (color == 0) { // R
            R = c;
            int dh = abs(l - r);
            int dv = abs(u - d);
            G = (dh < dv) ? ((l + r) >> 1) : (dv < dh) ? ((u + d) >> 1) : ((l + r + u + d) >> 2);
            B = (ul + ur + dl + dr) >> 2;
        } else if (color == 2) { // B
            B = c;
            int dh = abs(l - r);
            int dv = abs(u - d);
            G = (dh < dv) ? ((l + r) >> 1) : (dv < dh) ? ((u + d) >> 1) : ((l + r + u + d) >> 2);
            R = (ul + ur + dl + dr) >> 2;
        } else { // G
            G = c;
            if (g_on_r) {
                R = (l + r) >> 1;
                B = (u + d) >> 1;
            } else {
                R = (u + d) >> 1;
                B = (l + r) >> 1;
            }
        }
    } else {
        if (color == 0) { // R
            R = c;
            G = (l + r + u + d) >> 2;
            B = (ul + ur + dl + dr) >> 2;
        } else if (color == 2) { // B
            B = c;
            G = (l + r + u + d) >> 2;
            R = (ul + ur + dl + dr) >> 2;
        } else { // G
            G = c;
            if (g_on_r) {
                R = (l + r) >> 1;
                B = (u + d) >> 1;
            } else {
                R = (u + d) >> 1;
                B = (l + r) >> 1;
            }
        }
    }

    // write out (three bytes)
    int out_idx = (gy * width + gx) * 3;
    storeRGB<O>(&out[out_idx], R, G, B);
}

void launchBayerKernel(
    int cv_code,
    dim3 grid,
    dim3 block,
    cudaStream_t stream,
    const uint8_t* d_bayer,
    uint8_t* d_out,
    int w,
    int h
) {
    size_t shared_bytes = (block.x + 2) * (block.y + 2) * sizeof(uint8_t);
    // RGGB
    if (cv_code == cv::COLOR_BayerRG2RGB) {
        bayer_kernel_shared<BayerPattern::RGGB, OutputOrder::RGB, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerRG2BGR) {
        bayer_kernel_shared<BayerPattern::RGGB, OutputOrder::BGR, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerRG2RGB_EA) {
        bayer_kernel_shared<BayerPattern::RGGB, OutputOrder::RGB, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerRG2BGR_EA) {
        bayer_kernel_shared<BayerPattern::RGGB, OutputOrder::BGR, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    }

    // BGGR
    else if (cv_code == cv::COLOR_BayerBG2RGB)
    {
        bayer_kernel_shared<BayerPattern::BGGR, OutputOrder::RGB, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerBG2BGR) {
        bayer_kernel_shared<BayerPattern::BGGR, OutputOrder::BGR, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerBG2RGB_EA) {
        bayer_kernel_shared<BayerPattern::BGGR, OutputOrder::RGB, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerBG2BGR_EA) {
        bayer_kernel_shared<BayerPattern::BGGR, OutputOrder::BGR, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    }

    // GBRG
    else if (cv_code == cv::COLOR_BayerGB2RGB)
    {
        bayer_kernel_shared<BayerPattern::GBRG, OutputOrder::RGB, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerGB2BGR) {
        bayer_kernel_shared<BayerPattern::GBRG, OutputOrder::BGR, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerGB2RGB_EA) {
        bayer_kernel_shared<BayerPattern::GBRG, OutputOrder::RGB, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerGB2BGR_EA) {
        bayer_kernel_shared<BayerPattern::GBRG, OutputOrder::BGR, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    }

    // GRBG
    else if (cv_code == cv::COLOR_BayerGR2RGB)
    {
        bayer_kernel_shared<BayerPattern::GRBG, OutputOrder::RGB, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerGR2BGR) {
        bayer_kernel_shared<BayerPattern::GRBG, OutputOrder::BGR, InterpMode::BILINEAR>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerGR2RGB_EA) {
        bayer_kernel_shared<BayerPattern::GRBG, OutputOrder::RGB, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    } else if (cv_code == cv::COLOR_BayerGR2BGR_EA) {
        bayer_kernel_shared<BayerPattern::GRBG, OutputOrder::BGR, InterpMode::EA>
            <<<grid, block, shared_bytes, stream>>>(d_bayer, d_out, w, h, w);
    }

    else
    {
        throw std::runtime_error("Unsupported cv::COLOR_Bayer* conversion");
    }
}

void CudaBayer::throwIfCudaError(cudaError_t err, const char* msg) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(err));
    }
}

void CudaBayer::throwIfNppError(NppStatus st, const char* msg) {
    if (st != NPP_SUCCESS) {
        throw std::runtime_error(
            std::string(msg) + ": NppStatus=" + std::to_string(static_cast<int>(st))
        );
    }
}

CudaBayer::CudaBayer() {
    cudaError_t cerr = cudaStreamCreate(&stream_);
    if (cerr != cudaSuccess) {
        stream_ = nullptr;
        throw std::runtime_error(
            std::string("cudaStreamCreate failed: ") + cudaGetErrorString(cerr)
        );
    }
}

CudaBayer::~CudaBayer() {
    try {
        release();
    } catch (...) {
    }
    if (stream_) {
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
    }
}

void CudaBayer::release() {
    if (stream_)
        cudaStreamSynchronize(stream_);

    for (int i = 0; i < BUF_NUM; i++) {
        if (d_bayer_[i]) {
            cudaFree(d_bayer_[i]);
            d_bayer_[i] = nullptr;
        }
        if (d_target_[i]) {
            cudaFree(d_target_[i]);
            d_target_[i] = nullptr;
        }
        if (h_bayer_pinned_[i]) {
            cudaFreeHost(h_bayer_pinned_[i]);
            h_bayer_pinned_[i] = nullptr;
        }
        if (h_target_pinned_[i]) {
            cudaFreeHost(h_target_pinned_[i]);
            h_target_pinned_[i] = nullptr;
        }
        if (events_[i]) {
            cudaEventDestroy(events_[i]);
            events_[i] = nullptr;
        }
        d_bayer_pitch_[i] = 0;
        d_target_pitch_[i] = 0;
    }

    width_ = 0;
    height_ = 0;
    bayer_capacity_ = 0;
    rgb_capacity_ = 0;
}

void CudaBayer::allocIfNeeded(int width, int height) {
    if (width <= 0 || height <= 0)
        throw std::runtime_error("allocIfNeeded: invalid dims");

    size_t needed_bayer = static_cast<size_t>(width) * height;
    size_t needed_rgb = needed_bayer * 3;

    if (needed_bayer <= bayer_capacity_ && needed_rgb <= rgb_capacity_) {
        width_ = width;
        height_ = height;
        return;
    }

    release();

    for (int i = 0; i < BUF_NUM; i++) {
        cudaError_t cerr;
        // device memory (pitched)
        cerr = cudaMallocPitch(
            reinterpret_cast<void**>(&d_bayer_[i]),
            &d_bayer_pitch_[i],
            width,
            height
        );
        throwIfCudaError(cerr, "cudaMallocPitch d_bayer_ failed");
        cerr = cudaMallocPitch(
            reinterpret_cast<void**>(&d_target_[i]),
            &d_target_pitch_[i],
            width * 3,
            height
        );
        throwIfCudaError(cerr, "cudaMallocPitch d_target_ failed");

        // pinned host buffers
        cerr = cudaHostAlloc(
            reinterpret_cast<void**>(&h_bayer_pinned_[i]),
            needed_bayer,
            cudaHostAllocDefault
        );
        throwIfCudaError(cerr, "cudaHostAlloc h_bayer_pinned_ failed");
        cerr = cudaHostAlloc(
            reinterpret_cast<void**>(&h_target_pinned_[i]),
            needed_rgb,
            cudaHostAllocDefault
        );
        throwIfCudaError(cerr, "cudaHostAlloc h_target_pinned_ failed");

        // events
        cerr = cudaEventCreate(&events_[i]);
        throwIfCudaError(cerr, "cudaEventCreate failed");
    }

    width_ = width;
    height_ = height;
    bayer_capacity_ = needed_bayer;
    rgb_capacity_ = needed_rgb;
}

void CudaBayer::process(const cv::Mat& bayer, cv::Mat& target, int cv_bayer_code) {
    CV_Assert(bayer.type() == CV_8UC1 && bayer.isContinuous());
    int w = bayer.cols;
    int h = bayer.rows;
    if (w <= 0 || h <= 0)
        throw std::runtime_error("process: invalid image size");

    allocIfNeeded(w, h);
    size_t host_bayer_bytes = static_cast<size_t>(w) * h;
    size_t host_target_bytes = host_bayer_bytes * 3;

    int prevBuf = 1 - curBuf_;

    // 上一帧完成后拷贝到 target
    if (events_[prevBuf]) {
        cudaEventSynchronize(events_[prevBuf]);
        if (target.rows != h || target.cols != w || target.type() != CV_8UC3)
            target.create(h, w, CV_8UC3);
        std::memcpy(target.data, h_target_pinned_[prevBuf], host_target_bytes);
    }

    // copy Bayer to pinned host
    std::memcpy(h_bayer_pinned_[curBuf_], bayer.data, host_bayer_bytes);

    // map cv codes to NPP
    NppiBayerGridPosition grid;
    switch (cv_bayer_code) {
        case cv::COLOR_BayerRG2BGR:
            grid = NPPI_BAYER_RGGB;
            break;
        case cv::COLOR_BayerBG2BGR:
            grid = NPPI_BAYER_BGGR;
            break;
        case cv::COLOR_BayerGB2BGR:
            grid = NPPI_BAYER_GBRG;
            break;
        case cv::COLOR_BayerGR2BGR:
            grid = NPPI_BAYER_GRBG;
            break;
        case cv::COLOR_BayerRG2BGR_EA:
            grid = NPPI_BAYER_RGGB;
            break;
        case cv::COLOR_BayerBG2BGR_EA:
            grid = NPPI_BAYER_BGGR;
            break;
        case cv::COLOR_BayerGB2BGR_EA:
            grid = NPPI_BAYER_GBRG;
            break;
        case cv::COLOR_BayerGR2BGR_EA:
            grid = NPPI_BAYER_GRBG;
            break;
        default:
            throw std::runtime_error("Unsupported Bayer code");
    }

    // async H2D
    cudaError_t cerr = cudaMemcpy2DAsync(
        d_bayer_[curBuf_],
        d_bayer_pitch_[curBuf_],
        h_bayer_pinned_[curBuf_],
        static_cast<size_t>(w),
        w,
        h,
        cudaMemcpyHostToDevice,
        stream_
    );
    throwIfCudaError(cerr, "cudaMemcpy2DAsync H2D failed");

    // NPP context
    NppStreamContext ctx {};
    ctx.hStream = stream_;
    NppiSize roiSize { w, h };
    NppiRect roiRect { 0, 0, w, h };

    if (d_bayer_pitch_[curBuf_] > static_cast<size_t>(std::numeric_limits<int>::max())
        || d_target_pitch_[curBuf_] > static_cast<size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("pitch too large for NPP");

    NppStatus nppSt = nppiCFAToRGB_8u_C1C3R_Ctx(
        d_bayer_[curBuf_],
        static_cast<int>(d_bayer_pitch_[curBuf_]),
        roiSize,
        roiRect,
        d_target_[curBuf_],
        static_cast<int>(d_target_pitch_[curBuf_]),
        grid,
        NPPI_INTER_UNDEFINED,
        ctx
    );
    throwIfNppError(nppSt, "nppiCFAToRGB_8u_C1C3R_Ctx failed");

    // async D2H
    cerr = cudaMemcpy2DAsync(
        h_target_pinned_[curBuf_],
        static_cast<size_t>(w * 3),
        d_target_[curBuf_],
        d_target_pitch_[curBuf_],
        static_cast<size_t>(w * 3),
        h,
        cudaMemcpyDeviceToHost,
        stream_
    );
    throwIfCudaError(cerr, "cudaMemcpy2DAsync D2H failed");

    // record event for下一帧同步
    cudaEventRecord(events_[curBuf_], stream_);

    // switch buffer
    curBuf_ = prevBuf;
}
} // namespace cuda_cvt