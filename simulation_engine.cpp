#include "simulation_engine.h"
#include "coordinate_transform.h"
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void SimulationEngine::generateDataset(const AppConfig& config) {
    std::cout << "\n--- Generating Simulation Dataset ---" << std::endl;
    
    int N = config.sim.num_points;
    if (N < 3) N = 3;

    // 1. Setup Ground Truth Transform
    double s = config.sim.true_scale;
    Eigen::Vector3d t = config.sim.true_translation;
    
    // Convert YPR degrees to Rotation Matrix
    double yaw = config.sim.true_rotation_ypr.x() * M_PI / 180.0;
    double pitch = config.sim.true_rotation_ypr.y() * M_PI / 180.0;
    double roll = config.sim.true_rotation_ypr.z() * M_PI / 180.0;
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).matrix();

    // 2. Setup Origin for geographic conversion
    CoordinateTransform coord_trans;
    GPSPoint origin{39.908, 116.397, 50.0};
    coord_trans.setGPSOrigin(origin);

    // 3. Generators for points and noise
    std::vector<Eigen::Vector3d> gt_enu(N);
    for (int i = 0; i < N; ++i) {
        if (config.sim.type == "circle") {
            double angle = 2.0 * M_PI * i / N;
            double r = config.sim.radius;
            gt_enu[i] = Eigen::Vector3d(r * std::cos(angle), r * std::sin(angle), 0); // Flat circle
        } else { // "line"
            gt_enu[i] = Eigen::Vector3d(i * 10.0, 0, 0); // Spaced by 10m on X axis
        }
    }

    std::mt19937 gen(42); // Fixed seed for reproducibility
    std::normal_distribution<double> dist_noise(0.0, config.sim.noise_level);
    
    // Generate Outlier indices
    std::vector<bool> is_outlier(N, false);
    if (config.sim.outlier_count > 0) {
        std::uniform_int_distribution<int> dist_idx(0, N - 1);
        for(int k=0; k<config.sim.outlier_count; ++k) {
            is_outlier[dist_idx(gen)] = true;
        }
    }

    // 4. Generate and write GPS (with noise)
    std::string gps_file = "../data/sim_gps_trajectory.csv";
    std::ofstream fgps(gps_file);
    if (!fgps) {
        std::cerr << "Failed to open " << gps_file << " for writing." << std::endl;
        return;
    }
    
    int outlier_actual_count = 0;
    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d noisy_enu = gt_enu[i];
        if (config.sim.noise_level > 0.0001) {
            noisy_enu.x() += dist_noise(gen);
            noisy_enu.y() += dist_noise(gen);
            noisy_enu.z() += dist_noise(gen);
        }
        
        if (is_outlier[i]) {
            // Add a massive 50m jump to simulate outlier
            noisy_enu.x() += 50.0;
            noisy_enu.y() -= 50.0;
            outlier_actual_count++;
        }
        
        GPSPoint pt = coord_trans.ENUtoLLA(noisy_enu);
        fgps << std::fixed << pt.latitude << ", " << pt.longitude << ", " << pt.altitude << "\n";
    }
    fgps.close();
    std::cout << "[SIM] Created " << gps_file << " with " << N << " points." << std::endl;
    if (outlier_actual_count > 0) {
        std::cout << "[SIM] Injected " << outlier_actual_count << " outliers." << std::endl;
    }

    // 5. Generate and write COLMAP images.txt (clean, inverse transformed)
    std::string img_file = "../data/sim_images.txt";
    std::ofstream fimg(img_file);
    if (!fimg) {
        std::cerr << "Failed to open " << img_file << " for writing." << std::endl;
        return;
    }
    
    // Formula: ENU = s * R * SfM + t  ==>  SfM = (1/s) * R^T * (ENU - t)
    Eigen::Matrix3d R_inv = R.transpose();
    double s_inv = 1.0 / s;
    
    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d p_sfm = s_inv * R_inv * (gt_enu[i] - t);
        
        // Output format: IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        // We only care about camera center C, which is -R_colmap^T * T_colmap
        // Let's just output identity rotation (QW=1, others 0) for SfM pose
        // Then T_colmap = -R_colmap * C = -C.
        // Use 3-digit zero padding for filenames to ensure correct string sorting (frame_001, frame_002, etc.)
        double tx = -p_sfm.x();
        double ty = -p_sfm.y();
        double tz = -p_sfm.z();
        char buf[32];
        snprintf(buf, sizeof(buf), "frame_%03d.jpg", i);
        fimg << (i+1) << " 1.0 0.0 0.0 0.0 " << tx << " " << ty << " " << tz << " 1 " << buf << "\n";
        fimg << "0 0 0\n"; // Dummy points 2D
    }
    fimg.close();
    std::cout << "[SIM] Created " << img_file << " corresponding to Ground Truth." << std::endl;
}
