#include "alignment_solver.h"
#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <set>

// ============================================================
// 构造函数: 设置RANSAC默认参数
// ============================================================
AlignmentSolver::AlignmentSolver()
    : m_ransac_iterations(1000)
    , m_ransac_threshold(2.0)   // 2米以内算内点
    , m_min_inlier_ratio(0.5)   // 至少50%的点是内点才算有效
    , m_rng(42)                 // 固定种子, 保证结果可复现
{
}

// ============================================================
// Umeyama 算法: 从两组对应点中求解最优相似变换
// ============================================================
//
// 数学原理:
//   给定源点集 X (3xN) 和目标点集 Y (3xN),
//   寻找 scale(s), rotation(R), translation(t) 使得:
//     sum || Y_i - (s*R*X_i + t) ||^2 最小
//
//   步骤:
//     1. 计算两组点的质心 (均值)
//     2. 去质心化 (De-meaning)
//     3. 构建协方差矩阵 H = X_centered * Y_centered^T
//     4. 对 H 进行 SVD 分解: H = U * S * V^T
//     5. R = V * diag(1,1,det(V*U^T)) * U^T  (保证det(R)=+1)
//     6. 计算尺度因子 s
//     7. 计算平移 t = mean_Y - s * R * mean_X
//
AlignmentResult AlignmentSolver::solveUmeyama(
    const Eigen::MatrixXd& src,
    const Eigen::MatrixXd& dst,
    bool with_scale) const
{
    AlignmentResult result;
    const int N = static_cast<int>(src.cols());

    // Eigen 内置了 umeyama 函数, 直接返回 4x4 齐次变换矩阵
    // 参数: (源点, 目标点, 是否计算尺度)
    // 返回的矩阵格式:
    //   | s*R  t |
    //   |  0   1 |
    Eigen::Matrix4d T = Eigen::umeyama(src, dst, with_scale);

    result.transform = T;

    // 从 4x4 矩阵中提取各分量
    Eigen::Matrix3d sR = T.block<3, 3>(0, 0); // 左上角 3x3 = s*R
    result.translation = T.block<3, 1>(0, 3);  // 右上角 3x1 = t

    // 提取尺度: s = ||sR的第一列|| (因为R是正交矩阵, 每列模长为1)
    result.scale = sR.col(0).norm();

    // 提取旋转: R = sR / s
    if (result.scale > 1e-10) {
        result.rotation = sR / result.scale;
    } else {
        result.rotation = Eigen::Matrix3d::Identity();
    }

    // 计算对齐后的RMSE
    Eigen::MatrixXd projected = applyTransformBatch(result, src);
    result.rmse = calculateRMSE(projected, dst);

    result.total_count = N;
    result.inlier_count = N; // 非RANSAC模式, 所有点都算内点
    result.inlier_mask.assign(N, true);

    return result;
}

// ============================================================
// RANSAC + Umeyama: 鲁棒对齐算法
// ============================================================
//
// 为什么需要RANSAC?
//   GPS信号可能突然跳变几米甚至几十米(多径效应/信号遮挡),
//   如果直接用全部点做Umeyama, 这些异常点会严重拖偏结果。
//   RANSAC的策略是"随机抽样, 投票表决":
//     - 每次只用3个点算变换(最小样本集)
//     - 检查其余点是否也符合这个变换
//     - 取得票最多的那个变换作为最终结果
//
AlignmentResult AlignmentSolver::solveRANSAC(
    const Eigen::MatrixXd& src,
    const Eigen::MatrixXd& dst) const
{
    const int N = static_cast<int>(src.cols());
    const int min_samples = 3; // Umeyama至少需要3个非共线点

    if (N < min_samples) {
        std::cerr << "[RANSAC] Error: need at least 3 points, got " << N << std::endl;
        return solveUmeyama(src, dst);
    }

    AlignmentResult best_result;
    best_result.inlier_count = 0;
    best_result.rmse = 1e10;

    std::uniform_int_distribution<int> dist(0, N - 1);

    std::cout << "[RANSAC] Starting " << m_ransac_iterations
              << " iterations, threshold=" << m_ransac_threshold << "m" << std::endl;

    for (int iter = 0; iter < m_ransac_iterations; ++iter) {
        // Step 1: 随机抽取3个不重复的点
        std::set<int> indices_set;
        while (static_cast<int>(indices_set.size()) < min_samples) {
            indices_set.insert(dist(m_rng));
        }
        std::vector<int> indices(indices_set.begin(), indices_set.end());

        // 构建最小样本子集 (3x3 矩阵)
        Eigen::MatrixXd src_sample(3, min_samples);
        Eigen::MatrixXd dst_sample(3, min_samples);
        for (int i = 0; i < min_samples; ++i) {
            src_sample.col(i) = src.col(indices[i]);
            dst_sample.col(i) = dst.col(indices[i]);
        }

        // Step 2: 用最小样本计算候选变换
        AlignmentResult candidate = solveUmeyama(src_sample, dst_sample);

        // Step 3: 用候选变换评估所有点, 统计内点
        int inlier_count = 0;
        std::vector<bool> inlier_mask(N, false);

        for (int i = 0; i < N; ++i) {
            Eigen::Vector3d projected = applyTransform(candidate, src.col(i));
            double error = (projected - dst.col(i)).norm();
            if (error < m_ransac_threshold) {
                inlier_mask[i] = true;
                inlier_count++;
            }
        }

        // Step 4: 如果这个候选比之前的都好, 记录下来
        if (inlier_count > best_result.inlier_count) {
            best_result.inlier_count = inlier_count;
            best_result.inlier_mask = inlier_mask;
        }
    }

    // Step 5: 用所有内点重新做一次精确的Umeyama (全局精修)
    std::cout << "[RANSAC] Best inlier count: " << best_result.inlier_count
              << " / " << N << std::endl;

    if (best_result.inlier_count < min_samples) {
        std::cerr << "[RANSAC] Warning: too few inliers, falling back to full Umeyama" << std::endl;
        return solveUmeyama(src, dst);
    }

    // 提取内点子集
    Eigen::MatrixXd src_inliers(3, best_result.inlier_count);
    Eigen::MatrixXd dst_inliers(3, best_result.inlier_count);
    int idx = 0;
    for (int i = 0; i < N; ++i) {
        if (best_result.inlier_mask[i]) {
            src_inliers.col(idx) = src.col(i);
            dst_inliers.col(idx) = dst.col(i);
            idx++;
        }
    }

    // 用全部内点做最终的精确对齐
    AlignmentResult final_result = solveUmeyama(src_inliers, dst_inliers);
    final_result.inlier_count = best_result.inlier_count;
    final_result.total_count = N;
    final_result.inlier_mask = best_result.inlier_mask;

    // 重新计算基于全部点的RMSE (包括异常值)
    Eigen::MatrixXd all_projected = applyTransformBatch(final_result, src);
    final_result.rmse = calculateRMSE(all_projected, dst);

    // 也计算仅内点的RMSE
    Eigen::MatrixXd inlier_projected = applyTransformBatch(final_result, src_inliers);
    double inlier_rmse = calculateRMSE(inlier_projected, dst_inliers);

    std::cout << "[RANSAC] Inlier RMSE: " << inlier_rmse << " m" << std::endl;
    std::cout << "[RANSAC] Overall RMSE: " << final_result.rmse << " m" << std::endl;

    return final_result;
}

// ============================================================
// 工具函数
// ============================================================

Eigen::Vector3d AlignmentSolver::applyTransform(
    const AlignmentResult& result,
    const Eigen::Vector3d& src_point) const
{
    return result.scale * result.rotation * src_point + result.translation;
}

Eigen::MatrixXd AlignmentSolver::applyTransformBatch(
    const AlignmentResult& result,
    const Eigen::MatrixXd& src) const
{
    const int N = static_cast<int>(src.cols());
    Eigen::MatrixXd dst(3, N);
    for (int i = 0; i < N; ++i) {
        dst.col(i) = applyTransform(result, src.col(i));
    }
    return dst;
}

double AlignmentSolver::calculateRMSE(
    const Eigen::MatrixXd& a,
    const Eigen::MatrixXd& b) const
{
    // RMSE = sqrt( (1/N) * sum( ||a_i - b_i||^2 ) )
    const int N = static_cast<int>(a.cols());
    double sum_sq = 0.0;
    for (int i = 0; i < N; ++i) {
        sum_sq += (a.col(i) - b.col(i)).squaredNorm();
    }
    return std::sqrt(sum_sq / N);
}

double AlignmentSolver::extractScale(const Eigen::Matrix4d& T) {
    return T.block<3, 1>(0, 0).norm();
}

// ============================================================
// RANSAC 参数设置
// ============================================================

void AlignmentSolver::setRANSACIterations(int n) { m_ransac_iterations = n; }
void AlignmentSolver::setRANSACThreshold(double t) { m_ransac_threshold = t; }
void AlignmentSolver::setMinInlierRatio(double r) { m_min_inlier_ratio = r; }
void AlignmentSolver::setRandomSeed(unsigned int seed) { m_rng.seed(seed); }
