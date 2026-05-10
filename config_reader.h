#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// Stores all configuration parameters loaded from config.yaml
struct AppConfig {
    // Input paths
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
};

// Load configuration from a YAML file
// Returns AppConfig with all parameters populated
// Throws std::runtime_error if file cannot be opened or parsed
AppConfig loadConfig(const std::string& config_path);

#endif // CONFIG_READER_H
