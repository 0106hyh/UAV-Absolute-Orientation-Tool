#ifndef ALIGNMENT_SOLVER_H
#define ALIGNMENT_SOLVER_H

#include <Eigen/Dense>
#include <vector>
#include <random>

// ============================================================
// 对齐结果: 存储 Umeyama 算法计算出的相似变换参数
// ============================================================
// 变换关系: P_dst = scale * R * P_src + translation
// 即: GPS坐标 = 尺度 * 旋转 * SfM坐标 + 平移
struct AlignmentResult {
    double scale;               // 尺度因子 s (SfM无量纲 -> 真实米)
    Eigen::Matrix3d rotation;   // 3x3 旋转矩阵 R
    Eigen::Vector3d translation;// 3x1 平移向量 t
    Eigen::Matrix4d transform;  // 4x4 齐次变换矩阵 (包含 s, R, t)

    double rmse;                // 均方根误差 (米)
    int inlier_count;           // 内点数量 (被RANSAC认定为"好点"的数量)
    int total_count;            // 总点数
    std::vector<bool> inlier_mask; // 每个点是否为内点的标记
};

// ============================================================
// 对齐求解器: 核心算法类
// ============================================================
class AlignmentSolver {
public:
    AlignmentSolver();

    // ===================== 核心算法 =====================

    // 直接调用 Umeyama 算法 (不带RANSAC, 假设所有点都是好点)
    // src: 3xN 源点矩阵 (SfM坐标)
    // dst: 3xN 目标点矩阵 (GPS/ENU坐标)
    // with_scale: 是否计算尺度因子 (通常为true)
    AlignmentResult solveUmeyama(const Eigen::MatrixXd& src,
                                 const Eigen::MatrixXd& dst,
                                 bool with_scale = true) const;

    // 带RANSAC的鲁棒对齐 (能自动过滤GPS异常值)
    // src: 3xN 源点矩阵
    // dst: 3xN 目标点矩阵
    AlignmentResult solveRANSAC(const Eigen::MatrixXd& src,
                                const Eigen::MatrixXd& dst) const;

    // ===================== 工具函数 =====================

    // 用变换结果将源点投影到目标空间
    // 公式: P_projected = scale * R * P_src + t
    Eigen::Vector3d applyTransform(const AlignmentResult& result,
                                    const Eigen::Vector3d& src_point) const;

    // 批量投影
    Eigen::MatrixXd applyTransformBatch(const AlignmentResult& result,
                                         const Eigen::MatrixXd& src) const;

    // 计算两组点之间的RMSE (均方根误差)
    double calculateRMSE(const Eigen::MatrixXd& a,
                          const Eigen::MatrixXd& b) const;

    // 从齐次变换矩阵中提取尺度因子
    // 用途: 真实物理距离 = 模型欧氏距离 * scale
    static double extractScale(const Eigen::Matrix4d& T);

    // ===================== RANSAC 参数设置 =====================

    void setRANSACIterations(int n);       // 默认 1000
    void setRANSACThreshold(double t);     // 内点判定阈值 (米), 默认 2.0
    void setMinInlierRatio(double r);      // 最低内点比例, 默认 0.5
    void setRandomSeed(unsigned int seed); // 随机种子 (用于可重复实验)

private:
    int m_ransac_iterations;
    double m_ransac_threshold;
    double m_min_inlier_ratio;
    mutable std::mt19937 m_rng; // 随机数生成器
};

#endif // ALIGNMENT_SOLVER_H
