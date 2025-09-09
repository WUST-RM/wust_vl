#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace wust_vl_algorithm {

class SMC_MIMO {
public:
    /**
     * @param dim 控制维度
     * @param B 系统控制矩阵 (dim x dim)
     * @param lambda 滑模增益 (dim x dim 或对角矩阵)
     * @param k 到达律增益 (dim x dim 或对角矩阵)
     * @param phi_vec 边界层向量 (dim) 或标量，防止抖振
     * @param alpha 控制量低通滤波系数 (0~1)
     */
    SMC_MIMO(int dim, 
             const Eigen::MatrixXd& B,
             const Eigen::MatrixXd& lambda,
             const Eigen::MatrixXd& k,
             const Eigen::VectorXd& phi_vec,
             double alpha = 0.2)
        : dim_(dim), B_(B), Lambda_(lambda), K_(k), phi_vec_(phi_vec), alpha_(alpha)
    {
        if (B_.rows() != dim_ || B_.cols() != dim_ ||
            Lambda_.rows() != dim_ || Lambda_.cols() != dim_ ||
            K_.rows() != dim_ || K_.cols() != dim_ ||
            phi_vec_.size() != dim_) {
            throw std::runtime_error("Matrix/vector dimension mismatch!");
        }
        s_ = Eigen::VectorXd::Zero(dim_);
        u_filtered_ = Eigen::VectorXd::Zero(dim_);
    }

    /**
     * @brief 计算控制输入
     * @param x 当前状态向量
     * @param x_ref 期望状态向量
     * @param x_dot 当前状态导数向量
     * @return 控制输入向量
     */
    Eigen::VectorXd computeControl(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& x_ref,
        const Eigen::VectorXd& x_dot
    ) {
        if (x.size() != dim_ || x_ref.size() != dim_ || x_dot.size() != dim_) {
            throw std::runtime_error("State vector dimension mismatch!");
        }

        // 误差
        Eigen::VectorXd e = x - x_ref;
        Eigen::VectorXd e_dot = x_dot;

        // 滑模面 s = e_dot + Lambda * e
        s_ = e_dot + Lambda_ * e;

        // 连续饱和函数
        Eigen::VectorXd sat_s(dim_);
        for (int i = 0; i < dim_; ++i) {
            sat_s[i] = std::tanh(s_[i] / phi_vec_[i]);
        }

        // 原始 SMC 控制律
        Eigen::VectorXd u = -B_.inverse() * K_ * sat_s;

        // 低通滤波 u_filtered = alpha * u_prev + (1-alpha) * u_current
        u_filtered_ = alpha_ * u_filtered_ + (1.0 - alpha_) * u;

        return u_filtered_;
    }

    Eigen::VectorXd getSlidingSurface() const { return s_; }

    Eigen::VectorXd getFilteredControl() const { return u_filtered_; }

    void updateParameters(
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& Lambda,
        const Eigen::MatrixXd& K,
        const Eigen::VectorXd& phi_vec,
        double alpha = 0.2
    ) {
        if (B.rows() != dim_ || B.cols() != dim_ ||
            Lambda.rows() != dim_ || Lambda.cols() != dim_ ||
            K.rows() != dim_ || K.cols() != dim_ ||
            phi_vec.size() != dim_) {
            throw std::runtime_error("Matrix/vector dimension mismatch!");
        }
        B_ = B;
        Lambda_ = Lambda;
        K_ = K;
        phi_vec_ = phi_vec;
        alpha_ = alpha;
    }

private:
    int dim_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd Lambda_;
    Eigen::MatrixXd K_;
    Eigen::VectorXd phi_vec_;
    double alpha_;  // 控制量低通滤波系数

    Eigen::VectorXd s_;          // 滑模面
    Eigen::VectorXd u_filtered_; // 滤波后的控制量
};

} // namespace wust_vl_algorithm
