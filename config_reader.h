#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// Stores all configuration parameters loaded from config.yaml
struct AppConfig {
    // --- Global Mode ---
    std::string run_mode; // "work" or "sim"

    // --- Input paths ---
    std::string gps_file;
    std::string colmap_file;

    // ENU origin
    bool auto_origin;       // true = use first GPS point as origin
    double origin_lat;
    double origin_lon;
    double origin_alt;

    // RANSAC
    bool ransac_enabled;
    int ransac_iterations;
    double ransac_threshold;
    double min_inlier_ratio;

    // Output
    std::string transform_file;
    std::string trajectory_file;
    bool verbose;

    // Measurements
    struct PointPair { 
        Eigen::Vector3d p1; 
        Eigen::Vector3d p2; 
    };
    std::vector<PointPair> measurements;

    // --- Simulation Settings ---
    struct SimConfig {
        std::string type;       // "line" or "circle"
        int num_points;         // Number of points to generate
        double radius;          // Used if type == "circle"
        double noise_level;     // Standard deviation of GPS noise in meters
        
        // Ground Truth Transform (ENU = s * R * SfM + t)
        double true_scale;
        Eigen::Vector3d true_rotation_ypr; // Yaw, Pitch, Roll in degrees
        Eigen::Vector3d true_translation;  // ENU translation
        
        int outlier_count;      // Number of random outliers to inject
    } sim;
};

// Load configuration from a YAML file
// Returns AppConfig with all parameters populated
// Throws std::runtime_error if file cannot be opened or parsed
AppConfig loadConfig(const std::string& config_path);

#endif // CONFIG_READER_H
