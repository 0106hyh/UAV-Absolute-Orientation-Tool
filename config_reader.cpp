#include "config_reader.h"
#include <iostream>
#include <stdexcept>

AppConfig loadConfig(const std::string& config_path) {
    AppConfig cfg;

    YAML::Node root;
    try {
        root = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load config: " + std::string(e.what()));
    }

    // --- Input ---
    if (root["input"]) {
        cfg.gps_file = root["input"]["gps_file"].as<std::string>("");
        cfg.colmap_file = root["input"]["colmap_file"].as<std::string>("");
    }

    // --- ENU Origin ---
    if (root["enu_origin"]) {
        std::string mode = root["enu_origin"]["mode"].as<std::string>("auto");
        cfg.auto_origin = (mode == "auto");
        cfg.origin_lat = root["enu_origin"]["latitude"].as<double>(0.0);
        cfg.origin_lon = root["enu_origin"]["longitude"].as<double>(0.0);
        cfg.origin_alt = root["enu_origin"]["altitude"].as<double>(0.0);
    } else {
        cfg.auto_origin = true;
    }

    // --- RANSAC ---
    if (root["ransac"]) {
        cfg.ransac_enabled = root["ransac"]["enabled"].as<bool>(true);
        cfg.ransac_iterations = root["ransac"]["iterations"].as<int>(1000);
        cfg.ransac_threshold = root["ransac"]["threshold"].as<double>(2.0);
        cfg.min_inlier_ratio = root["ransac"]["min_inlier_ratio"].as<double>(0.5);
    } else {
        cfg.ransac_enabled = true;
        cfg.ransac_iterations = 1000;
        cfg.ransac_threshold = 2.0;
        cfg.min_inlier_ratio = 0.5;
    }

    // --- Output ---
    if (root["output"]) {
        cfg.transform_file = root["output"]["transform_file"].as<std::string>("transform_result.txt");
        cfg.trajectory_file = root["output"]["trajectory_file"].as<std::string>("aligned_trajectory.csv");
        cfg.verbose = root["output"]["verbose"].as<bool>(true);
    } else {
        cfg.transform_file = "transform_result.txt";
        cfg.trajectory_file = "aligned_trajectory.csv";
        cfg.verbose = true;
    }

    // --- Measurements ---
    if (root["measurements"]) {
        for (const auto& node : root["measurements"]) {
            AppConfig::PointPair pair;
            pair.p1 = Eigen::Vector3d(node["p1"][0].as<double>(), node["p1"][1].as<double>(), node["p1"][2].as<double>());
            pair.p2 = Eigen::Vector3d(node["p2"][0].as<double>(), node["p2"][1].as<double>(), node["p2"][2].as<double>());
            cfg.measurements.push_back(pair);
        }
    }

    std::cout << "[Config] Loaded from: " << config_path << std::endl;
    std::cout << "[Config] GPS file:    " << cfg.gps_file << std::endl;
    std::cout << "[Config] COLMAP file: " << cfg.colmap_file << std::endl;
    std::cout << "[Config] RANSAC:      " << (cfg.ransac_enabled ? "ON" : "OFF")
              << " (iter=" << cfg.ransac_iterations
              << ", thr=" << cfg.ransac_threshold << "m)" << std::endl;

    return cfg;
}
