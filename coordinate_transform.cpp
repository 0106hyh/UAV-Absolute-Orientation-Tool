#include "coordinate_transform.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// ============================================================
// 构造函数
// ============================================================
CoordinateTransform::CoordinateTransform()
    : m_projector(0, 0, 0) // 初始化为零点，稍后由 setGPSOrigin 覆盖
      ,
      m_origin_set(false) {}

// ============================================================
// GPS 坐标转换部分
// ============================================================

void CoordinateTransform::setGPSOrigin(const GPSPoint &origin) {
  // 使用 GeographicLib 的 LocalCartesian 对象
  // 以 origin 的经纬高作为 ENU 坐标系的原点
  // 之后所有点的转换都相对于这个原点
  m_projector.Reset(origin.latitude, origin.longitude, origin.altitude);
  m_origin_set = true;

  std::cout << "[CoordinateTransform] ENU原点已设置: "
            << "lat=" << origin.latitude << ", "
            << "lon=" << origin.longitude << ", "
            << "alt=" << origin.altitude << " m" << std::endl;
}

Eigen::Vector3d CoordinateTransform::LLAtoENU(const GPSPoint &point) const {
  if (!m_origin_set) {
    throw std::runtime_error(
        "错误: 调用 LLAtoENU 前必须先调用 setGPSOrigin 设置参考原点!");
  }

  double east, north, up;
  // GeographicLib::LocalCartesian::Forward()
  //   输入: 目标点的 纬度、经度、高度
  //   输出: 该点相对于原点的 东(E)、北(N)、上(U) 坐标，单位为米
  //
  // 内部转换路径:
  //   LLA → ECEF(地心地固坐标) → 减去原点ECEF → 乘以旋转矩阵 → ENU
  m_projector.Forward(point.latitude, point.longitude, point.altitude, east,
                      north, up);

  return Eigen::Vector3d(east, north, up);
}

Eigen::MatrixXd CoordinateTransform::buildTargetMatrix(
    const std::vector<GPSPoint> &gps_points) const {
  const int N = static_cast<int>(gps_points.size());
  // 构建 3×N 矩阵，每一列存储一个GPS点的ENU坐标
  Eigen::MatrixXd Y(3, N);

  for (int i = 0; i < N; ++i) {
    Y.col(i) = LLAtoENU(gps_points[i]);
  }

  std::cout << "[CoordinateTransform] 目标点集 Y 构建完成: 3 x " << N
            << std::endl;
  return Y;
}

// ============================================================
// COLMAP 位姿转换部分
// ============================================================

Eigen::Vector3d
CoordinateTransform::extractCameraCenter(const ColmapPose &pose) const {
  // 步骤1: 将四元数转换为 3×3 旋转矩阵 R
  // Eigen::Quaterniond 的构造顺序是 (w, x, y, z)
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  Eigen::Matrix3d R = q.toRotationMatrix();

  // 步骤2: 构建平移向量 T
  Eigen::Vector3d T(pose.tx, pose.ty, pose.tz);

  // 步骤3: 计算相机光心的绝对位置
  // *** 核心逆变换公式: C = -R^T * T ***
  //
  // 为什么？
  //   COLMAP 中的外参定义了映射: P_camera = R * P_world + T
  //   即 "世界点 → 相机坐标系下的点"
  //
  //   相机中心在相机坐标系下就是原点 (0, 0, 0)
  //   所以: 0 = R * C_world + T
  //   解方程: C_world = -R^(-1) * T = -R^T * T
  //   (因为旋转矩阵是正交矩阵, R的逆等于R的转置)
  Eigen::Vector3d camera_center = -R.transpose() * T;

  return camera_center;
}

Eigen::MatrixXd CoordinateTransform::buildSourceMatrix(
    const std::vector<ColmapPose> &poses) const {
  const int N = static_cast<int>(poses.size());
  // 构建 3×N 矩阵，每一列存储一个相机光心的SfM坐标
  Eigen::MatrixXd X(3, N);

  for (int i = 0; i < N; ++i) {
    X.col(i) = extractCameraCenter(poses[i]);
  }

  std::cout << "[CoordinateTransform] 源点集 X 构建完成: 3 x " << N
            << std::endl;
  return X;
}

// ============================================================
// 数据读取工具
// ============================================================

std::vector<GPSPoint>
CoordinateTransform::loadGPSFromCSV(const std::string &filepath) {
  std::vector<GPSPoint> points;
  std::ifstream file(filepath);

  if (!file.is_open()) {
    throw std::runtime_error("错误: 无法打开GPS文件: " + filepath);
  }

  std::string line;
  int line_num = 0;

  while (std::getline(file, line)) {
    line_num++;

    // 跳过空行和注释行
    if (line.empty() || line[0] == '#')
      continue;

    std::istringstream iss(line);
    GPSPoint pt;
    char comma;

    // 预期格式: latitude, longitude, altitude
    if (iss >> pt.latitude >> comma >> pt.longitude >> comma >> pt.altitude) {
      points.push_back(pt);
    } else {
      std::cerr << "[警告] 第 " << line_num << " 行格式错误, 已跳过: " << line
                << std::endl;
    }
  }

  std::cout << "[CoordinateTransform] 从 CSV 文件加载了 " << points.size()
            << " 个GPS点" << std::endl;
  return points;
}

std::vector<ColmapPose>
CoordinateTransform::loadColmapPoses(const std::string &filepath) {
  std::vector<ColmapPose> poses;
  std::ifstream file(filepath);

  if (!file.is_open()) {
    throw std::runtime_error("错误: 无法打开COLMAP文件: " + filepath);
  }

  std::string line;
  bool skip_next =
      false; // COLMAP的images.txt中，每个图像占两行，第二行是2D特征点

  while (std::getline(file, line)) {
    // 跳过注释行 (以 '#' 开头)
    if (line.empty() || line[0] == '#')
      continue;

    if (skip_next) {
      // 这是特征点行 (POINTS2D[])，跳过
      skip_next = false;
      continue;
    }

    // 解析位姿行
    // 格式: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID IMAGE_NAME
    std::istringstream iss(line);
    ColmapPose pose;
    int camera_id;

    if (iss >> pose.image_id >> pose.qw >> pose.qx >> pose.qy >> pose.qz >>
        pose.tx >> pose.ty >> pose.tz >> camera_id >> pose.image_name) {
      poses.push_back(pose);
    } else {
      std::cerr << "[警告] COLMAP位姿行解析失败: " << line << std::endl;
    }

    // 下一行是特征点，需要跳过
    skip_next = true;
  }

  std::cout << "[CoordinateTransform] 从 images.txt 加载了 " << poses.size()
            << " 个相机位姿" << std::endl;
  return poses;
}
