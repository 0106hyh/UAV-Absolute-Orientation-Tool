// ============================================================
// UAV 3D Reconstruction - Absolute Orientation Tool
// ============================================================
// Usage: uav_orientation.exe [config.yaml]
//
// This program reads GPS and COLMAP data, aligns them using
// Umeyama + RANSAC, and outputs the transformation matrix.

#include "coordinate_transform.h"
#include "alignment_solver.h"
#include "config_reader.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>
#include <sstream>

// ============================================================
// Helper: Load Scale from previous result file
// ============================================================
double loadScaleFromTransformFile(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) return -1.0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("scale:", 0) == 0) {
            return std::stod(line.substr(6));
        }
    }
    return -1.0;
}

// ============================================================
// Result Export Functions
// ============================================================

// Save transformation parameters and measurements to text file
void saveTransformResult(const std::string& filepath,
                         const AlignmentResult& result,
                         const AppConfig& cfg) {
    // Create output directory if needed
    std::filesystem::path p(filepath);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }

    std::ofstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "[Error] Cannot write to: " << filepath << std::endl;
        return;
    }

    f << std::fixed << std::setprecision(10);
    f << "# UAV 3D Alignment Result" << std::endl;
    f << "# Format: Similarity Transform  P_gps = scale * R * P_sfm + t" << std::endl;
    f << std::endl;

    f << "scale: " << result.scale << std::endl;
    f << std::endl;

    f << "rotation:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        f << "  " << result.rotation(i, 0) << " "
                   << result.rotation(i, 1) << " "
                   << result.rotation(i, 2) << std::endl;
    }
    f << std::endl;

    f << "translation: " << result.translation.x() << " "
                         << result.translation.y() << " "
                         << result.translation.z() << std::endl;
    f << std::endl;

    f << "rmse: " << result.rmse << std::endl;
    f << "inliers: " << result.inlier_count << " / " << result.total_count << std::endl;
    f << std::endl;

    // --- Save Measurements ---
    if (!cfg.measurements.empty()) {
        f << "# Distance Measurements" << std::endl;
        for (size_t i = 0; i < cfg.measurements.size(); ++i) {
            const auto& pair = cfg.measurements[i];
            double model_dist = (pair.p1 - pair.p2).norm();
            double real_dist = model_dist * result.scale;
            f << "measurement_" << i << ":" << std::endl;
            f << "  p1: [" << pair.p1.transpose() << "]" << std::endl;
            f << "  p2: [" << pair.p2.transpose() << "]" << std::endl;
            f << "  model_distance: " << model_dist << std::endl;
            f << "  real_distance_meters: " << real_dist << std::endl;
        }
    }

    f.close();
    std::cout << "[Output] Transform saved to: " << filepath << std::endl;
}

// Save aligned trajectory as CSV
void saveTrajectory(const std::string& filepath,
                    const AlignmentResult& result,
                    const std::vector<ColmapPose>& poses,
                    const AlignmentSolver& solver) {
    std::filesystem::path p(filepath);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }

    std::ofstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "[Error] Cannot write to: " << filepath << std::endl;
        return;
    }

    CoordinateTransform ct_tmp;
    f << "image_name,enu_east,enu_north,enu_up" << std::endl;
    f << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < poses.size(); ++i) {
        Eigen::Vector3d sfm_center = ct_tmp.extractCameraCenter(poses[i]);
        Eigen::Vector3d enu_pos = solver.applyTransform(result, sfm_center);
        f << poses[i].image_name << ","
          << enu_pos.x() << "," << enu_pos.y() << "," << enu_pos.z()
          << std::endl;
    }

    f.close();
    std::cout << "[Output] Trajectory saved to: " << filepath << std::endl;
}

// ============================================================
// Data Matching: pair GPS points with COLMAP poses by image name
// ============================================================
struct MatchedPair {
    int gps_index;
    int pose_index;
};

// Simple matching: assume GPS CSV rows correspond to sorted image names
std::vector<MatchedPair> matchByOrder(
    const std::vector<GPSPoint>& gps,
    const std::vector<ColmapPose>& poses)
{
    std::vector<MatchedPair> pairs;
    int n = std::min(static_cast<int>(gps.size()),
                     static_cast<int>(poses.size()));

    // Sort poses by image_name to ensure consistent ordering
    std::vector<int> sorted_indices(poses.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](int a, int b) {
                  return poses[a].image_name < poses[b].image_name;
              });

    for (int i = 0; i < n; ++i) {
        pairs.push_back({i, sorted_indices[i]});
    }

    return pairs;
}

// ============================================================
// Main Program
// ============================================================
int main(int argc, char* argv[]) {
    std::cout << "============================================" << std::endl;
    std::cout << "  UAV 3D Absolute Orientation Tool" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::fixed << std::setprecision(6) << std::endl;

    // --- Step 1: Load Configuration ---
    std::string config_path = "config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    AppConfig cfg;
    try {
        cfg = loadConfig(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::cerr << "Usage: uav_orientation.exe [config.yaml]" << std::endl;
        return 1;
    }

    // --- Step 1.5: Snapshot Mode (Measurement Only) ---
    if (cfg.gps_file.empty() || cfg.gps_file == "none" || 
        cfg.colmap_file.empty() || cfg.colmap_file == "none") {
        
        std::cout << "\n============================================" << std::endl;
        std::cout << "  SNAPSHOT MODE (Measurement Only)" << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << "[Info] Skipping full alignment..." << std::endl;
        
        double scale = loadScaleFromTransformFile(cfg.transform_file);
        if (scale < 0) {
            std::cerr << "[Error] Cannot read scale from: " << cfg.transform_file << std::endl;
            std::cerr << "Please run full alignment at least once first." << std::endl;
            return 1;
        }
        std::cout << "[Info] Loaded scale from previous result: " << scale << std::endl;
        
        if (!cfg.measurements.empty()) {
            std::cout << "\n--- Distance Measurements ---" << std::endl;
            
            // Append new measurements to the existing result file
            std::ofstream f(cfg.transform_file, std::ios::app);
            if (f.is_open()) f << "\n# Snapshot Measurements" << std::endl;

            for (size_t i = 0; i < cfg.measurements.size(); ++i) {
                const auto& pair = cfg.measurements[i];
                double model_dist = (pair.p1 - pair.p2).norm();
                double real_dist = model_dist * scale;
                
                std::cout << "  [" << i << "] P1: (" << pair.p1.transpose() << ")" << std::endl;
                std::cout << "      P2: (" << pair.p2.transpose() << ")" << std::endl;
                std::cout << "      Model dist: " << model_dist << std::endl;
                std::cout << "      REAL DIST:  " << real_dist << " meters" << std::endl;
                std::cout << "      --------------------------" << std::endl;

                if (f.is_open()) {
                    f << "snapshot_measurement_" << i << ":" << std::endl;
                    f << "  p1: [" << pair.p1.transpose() << "]" << std::endl;
                    f << "  p2: [" << pair.p2.transpose() << "]" << std::endl;
                    f << "  model_distance: " << model_dist << std::endl;
                    f << "  real_distance_meters: " << real_dist << std::endl;
                }
            }
            if (f.is_open()) f.close();
            std::cout << "[Output] Measurements appended to: " << cfg.transform_file << std::endl;
        } else {
            std::cout << "[Info] No measurements found in config.yaml." << std::endl;
        }
        
        std::cout << "============================================" << std::endl;
        return 0;
    }

    // --- Step 2: Load GPS Data ---
    std::cout << "\n--- Loading GPS Data ---" << std::endl;
    std::vector<GPSPoint> gps_points;
    try {
        gps_points = CoordinateTransform::loadGPSFromCSV(cfg.gps_file);
    } catch (const std::exception& e) {
        std::cerr << "[Error] GPS loading failed: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "[GPS] Loaded " << gps_points.size() << " points" << std::endl;

    if (gps_points.size() < 3) {
        std::cerr << "[Error] Need at least 3 GPS points for alignment" << std::endl;
        return 1;
    }

    // --- Step 3: Load COLMAP Poses ---
    std::cout << "\n--- Loading COLMAP Data ---" << std::endl;
    std::vector<ColmapPose> colmap_poses;
    try {
        colmap_poses = CoordinateTransform::loadColmapPoses(cfg.colmap_file);
    } catch (const std::exception& e) {
        std::cerr << "[Error] COLMAP loading failed: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "[COLMAP] Loaded " << colmap_poses.size() << " poses" << std::endl;

    // --- Step 4: Match GPS & COLMAP Data ---
    std::cout << "\n--- Matching Data ---" << std::endl;
    std::vector<MatchedPair> matches = matchByOrder(gps_points, colmap_poses);
    std::cout << "[Match] Paired " << matches.size() << " points" << std::endl;

    if (matches.size() < 3) {
        std::cerr << "[Error] Need at least 3 matched pairs" << std::endl;
        return 1;
    }

    // --- Step 5: Coordinate Conversion (LLA -> ENU) ---
    std::cout << "\n--- Coordinate Conversion ---" << std::endl;
    CoordinateTransform transform;

    if (cfg.auto_origin) {
        transform.setGPSOrigin(gps_points[matches[0].gps_index]);
    } else {
        GPSPoint manual_origin = {cfg.origin_lat, cfg.origin_lon, cfg.origin_alt};
        transform.setGPSOrigin(manual_origin);
    }

    // Build target matrix Y (GPS in ENU) and source matrix X (SfM camera centers)
    const int N = static_cast<int>(matches.size());
    Eigen::MatrixXd Y(3, N); // GPS (target)
    Eigen::MatrixXd X(3, N); // SfM (source)

    for (int i = 0; i < N; ++i) {
        Y.col(i) = transform.LLAtoENU(gps_points[matches[i].gps_index]);
        X.col(i) = transform.extractCameraCenter(colmap_poses[matches[i].pose_index]);
    }

    if (cfg.verbose) {
        std::cout << "\n  First 5 matched pairs (ENU vs SfM):" << std::endl;
        for (int i = 0; i < std::min(5, N); ++i) {
            std::cout << "  [" << i << "] GPS_ENU=" << Y.col(i).transpose()
                      << "  SfM=" << X.col(i).transpose() << std::endl;
        }
    }

    // --- Step 6: Alignment ---
    std::cout << "\n--- Running Alignment ---" << std::endl;
    AlignmentSolver solver;
    AlignmentResult result;

    if (cfg.ransac_enabled) {
        solver.setRANSACIterations(cfg.ransac_iterations);
        solver.setRANSACThreshold(cfg.ransac_threshold);
        solver.setMinInlierRatio(cfg.min_inlier_ratio);
        result = solver.solveRANSAC(X, Y);
    } else {
        result = solver.solveUmeyama(X, Y);
    }

    // --- Step 7: Print Results ---
    std::cout << "\n============================================" << std::endl;
    std::cout << "  Alignment Results" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "  Scale factor:   " << result.scale << std::endl;
    std::cout << "  RMSE:           " << result.rmse << " m" << std::endl;
    std::cout << "  Inliers:        " << result.inlier_count
              << " / " << result.total_count << std::endl;
    std::cout << "\n  Rotation matrix:" << std::endl;
    std::cout << result.rotation << std::endl;
    std::cout << "\n  Translation (m): " << result.translation.transpose() << std::endl;

    // --- Step 8: Save Results ---
    std::cout << "\n--- Saving Results ---" << std::endl;
    saveTransformResult(cfg.transform_file, result, cfg);
    saveTrajectory(cfg.trajectory_file, result, colmap_poses, solver);

    // --- Step 9: Custom Measurements ---
    if (!cfg.measurements.empty()) {
        std::cout << "\n--- Distance Measurements ---" << std::endl;
        for (size_t i = 0; i < cfg.measurements.size(); ++i) {
            const auto& pair = cfg.measurements[i];
            double model_dist = (pair.p1 - pair.p2).norm();
            double real_dist = model_dist * result.scale;
            
            std::cout << "  [" << i << "] P1: (" << pair.p1.transpose() << ")" << std::endl;
            std::cout << "      P2: (" << pair.p2.transpose() << ")" << std::endl;
            std::cout << "      Model dist: " << model_dist << std::endl;
            std::cout << "      REAL DIST:  " << real_dist << " meters" << std::endl;
            std::cout << "      --------------------------" << std::endl;
        }
    }

    std::cout << "\n============================================" << std::endl;
    std::cout << "  Done! Check output files for details." << std::endl;
    std::cout << "============================================" << std::endl;

    return 0;
}
