#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

/**
 * @file coordinate_transform.h
 * @brief 坐标转换模块 —— 3D绝对定向的数据预处理层
 *
 * 本模块负责两件核心工作：
 * 1. 将GPS的经纬高(LLA)转换为局部笛卡尔坐标(ENU)，使所有数据单位统一为"米"
 * 2. 将COLMAP输出的相机外参(R, T)逆变换为相机在世界坐标系下的绝对位置
 *
 * 最终产出：两个 3×N 的点集矩阵，供 Umeyama 算法进行相似变换拟合
 */

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <string>
#include <vector>

/**
 * @struct GPSPoint
 * @brief 存储单个GPS测量点的经纬高数据 (WGS84 大地坐标系)
 */
struct GPSPoint {
  double latitude;  // 纬度 (度)
  double longitude; // 经度 (度)
  double altitude;  // 椭球高 (米)
};

/**
 * @struct ColmapPose
 * @brief 存储COLMAP输出的单张图像的相机位姿
 *
 * COLMAP的 images.txt 中，每张图的位姿由四元数和平移向量组成，
 * 它们表示的是"世界坐标系到相机坐标系"的变换，
 * 并非相机在世界中的位置！
 */
struct ColmapPose {
  int image_id;           // 图像ID
  double qw, qx, qy, qz;  // 四元数 (世界→相机 的旋转)
  double tx, ty, tz;      // 平移向量 (世界→相机 的平移)
  int camera_id;          // 相机ID
  std::string image_name; // 图像文件名（用于时间戳匹配）
};

/**
 * @class CoordinateTransform
 * @brief 坐标转换核心类
 *
 * 使用方法：
 *   1. 调用 setGPSOrigin() 设定ENU原点（通常取无人机起飞点）
 *   2. 调用 LLAtoENU() 将GPS数据批量转换为局部坐标
 *   3. 调用 extractCameraCenters() 从COLMAP位姿中提取绝对相机位置
 */
class CoordinateTransform {
public:
  CoordinateTransform();

  // ===================== GPS 坐标转换 =====================

  /**
   * @brief 设置ENU坐标系的参考原点
   * @param origin 参考原点的GPS坐标（通常取轨迹的第一个点）
   *
   * 所有后续的LLA坐标都将相对于这个原点进行转换。
   * 原点在ENU系下的坐标为 (0, 0, 0)。
   */
  void setGPSOrigin(const GPSPoint &origin);

  /**
   * @brief 将单个GPS点从LLA转换为ENU坐标
   * @param point GPS测量点 (经纬高)
   * @return 该点在ENU坐标系下的三维坐标 (米)
   *
   * 内部转换路径: WGS84(LLA) → ECEF → ENU
   * 使用 GeographicLib 保证纳米级精度
   */
  Eigen::Vector3d LLAtoENU(const GPSPoint &point) const;

  /**
   * @brief 将单个ENU点反向转换为LLA坐标 (用于仿真)
   * @param enu ENU坐标 (米)
   * @return 该点对应的GPS测量点 (经纬高)
   */
  GPSPoint ENUtoLLA(const Eigen::Vector3d &enu) const;

  /**
   * @brief 批量转换GPS点集，构建目标点矩阵 Y
   * @param gps_points GPS点的向量
   * @return 3×N 的 Eigen 矩阵，每列为一个ENU坐标
   *
   * 这就是 Umeyama 算法的"目标点集 Y"——真实世界的物理坐标
   */
  Eigen::MatrixXd
  buildTargetMatrix(const std::vector<GPSPoint> &gps_points) const;

  // ===================== COLMAP 位姿转换 =====================

  /**
   * @brief 从COLMAP外参中提取相机在世界坐标系下的绝对位置
   * @param pose COLMAP输出的相机位姿 (四元数 + 平移)
   * @return 相机光心在SfM世界坐标系下的三维坐标
   *
   * 核心公式: C_sfm = -R^T * T
   *
   * 为什么不能直接用 (tx, ty, tz)？
   *   因为COLMAP记录的 T 是"世界原点在相机坐标系下的位置"，
   *   而不是"相机在世界中的位置"。必须做逆变换才能得到相机中心。
   */
  Eigen::Vector3d extractCameraCenter(const ColmapPose &pose) const;

  /**
   * @brief 批量提取相机中心，构建源点矩阵 X
   * @param poses COLMAP位姿的向量
   * @return 3×N 的 Eigen 矩阵，每列为一个SfM空间下的相机中心坐标
   *
   * 这就是 Umeyama 算法的"源点集 X"——无量纲的视觉模型坐标
   */
  Eigen::MatrixXd buildSourceMatrix(const std::vector<ColmapPose> &poses) const;

  // ===================== 数据读取工具 =====================

  /**
   * @brief 从CSV文件读取GPS数据
   * @param filepath CSV文件路径 (格式: latitude, longitude, altitude)
   * @return GPS点的向量
   */
  static std::vector<GPSPoint> loadGPSFromCSV(const std::string &filepath);

  /**
   * @brief 解析COLMAP的images.txt文件
   * @param filepath images.txt文件路径
   * @return COLMAP位姿的向量
   *
   * images.txt格式说明：
   *   - 以 '#' 开头的行为注释
   *   - 每个图像占两行: 第一行是位姿，第二行是2D特征点(我们跳过)
   *   - 位姿行格式: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID IMAGE_NAME
   */
  static std::vector<ColmapPose> loadColmapPoses(const std::string &filepath);

private:
  GeographicLib::LocalCartesian m_projector; // GeographicLib 的局部坐标投影器
  bool m_origin_set;                         // 标记是否已设置原点
};

#endif // COORDINATE_TRANSFORM_H
