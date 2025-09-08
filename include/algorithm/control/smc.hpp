#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
namespace wust_vl_algorithm {
class SMC_MIMO {
public:
    /**
     * @param dim 控制维度
     * @param B 系统控制矩阵 (dim x dim)
     * @param lambda 滑模增益 (dim x dim 或对角矩阵)
     * @param k 到达律增益 (dim x dim 或对角矩阵)
     * @param phi 饱和函数边界，防止抖振
     */
    SMC_MIMO(int dim, 
             const Eigen::MatrixXd& B,
             const Eigen::MatrixXd& lambda,
             const Eigen::MatrixXd& k,
             double phi = 0.01)
        : dim_(dim), B_(B), Lambda_(lambda), K_(k), phi_(phi)
    {
        if (B_.rows() != dim_ || B_.cols() != dim_ ||
            Lambda_.rows() != dim_ || Lambda_.cols() != dim_ ||
            K_.rows() != dim_ || K_.cols() != dim_) {
            throw std::runtime_error("Matrix dimension mismatch!");
        }
        s_ = Eigen::VectorXd::Zero(dim_);
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

        // 控制律 u = - B^-1 * K * sat(s / phi)
        Eigen::VectorXd sat_s(dim_);
        for (int i = 0; i < dim_; ++i) {
            sat_s[i] = sat(s_[i] / phi_);
        }

        Eigen::VectorXd u = -B_.inverse() * K_ * sat_s;
        return u;
    }

    Eigen::VectorXd getSlidingSurface() const {
        return s_;
    }

private:
    int dim_;
    double phi_;  // sat函数边界
    Eigen::MatrixXd B_;
    Eigen::MatrixXd Lambda_;
    Eigen::MatrixXd K_;
    Eigen::VectorXd s_;

    inline double sat(double val) const {
        if (val > 1.0) return 1.0;
        if (val < -1.0) return -1.0;
        return val;
    }
};
}