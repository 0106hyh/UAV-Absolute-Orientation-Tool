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

    // --- Global Mode ---
    cfg.run_mode = root["run_mode"] ? root["run_mode"].as<std::string>("work") : "work";

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

    // --- Simulation ---
    if (root["simulation"]) {
        cfg.sim.type = root["simulation"]["type"].as<std::string>("circle");
        cfg.sim.num_points = root["simulation"]["num_points"].as<int>(20);
        cfg.sim.radius = root["simulation"]["radius"].as<double>(30.0);
        cfg.sim.noise_level = root["simulation"]["noise_level"].as<double>(0.0);
        
        if (root["simulation"]["ground_truth"]) {
            cfg.sim.true_scale = root["simulation"]["ground_truth"]["scale"].as<double>(1.0);
            
            auto ypr = root["simulation"]["ground_truth"]["rotation_ypr"];
            cfg.sim.true_rotation_ypr = Eigen::Vector3d(ypr[0].as<double>(), ypr[1].as<double>(), ypr[2].as<double>());
            
            auto trans = root["simulation"]["ground_truth"]["translation"];
            cfg.sim.true_translation = Eigen::Vector3d(trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>());
        } else {
            cfg.sim.true_scale = 1.0;
            cfg.sim.true_rotation_ypr = Eigen::Vector3d::Zero();
            cfg.sim.true_translation = Eigen::Vector3d::Zero();
        }
        cfg.sim.outlier_count = root["simulation"]["outlier_count"].as<int>(0);
    } else {
        cfg.sim.type = "circle";
        cfg.sim.num_points = 20;
        cfg.sim.radius = 30.0;
        cfg.sim.noise_level = 0.0;
        cfg.sim.true_scale = 1.0;
        cfg.sim.true_rotation_ypr = Eigen::Vector3d::Zero();
        cfg.sim.true_translation = Eigen::Vector3d::Zero();
        cfg.sim.outlier_count = 0;
    }

    std::cout << "[Config] Loaded from: " << config_path << std::endl;
    std::cout << "[Config] Run Mode:    " << (cfg.run_mode == "sim" ? "SIMULATION" : "WORK") << std::endl;
    
    if (cfg.run_mode != "sim") {
        std::cout << "[Config] GPS file:    " << cfg.gps_file << std::endl;
        std::cout << "[Config] COLMAP file: " << cfg.colmap_file << std::endl;
    } else {
        std::cout << "[Config] Sim Type:    " << cfg.sim.type << " (N=" << cfg.sim.num_points << ")" << std::endl;
    }
    
    std::cout << "[Config] RANSAC:      " << (cfg.ransac_enabled ? "ON" : "OFF")
              << " (iter=" << cfg.ransac_iterations
              << ", thr=" << cfg.ransac_threshold << "m)" << std::endl;

    return cfg;
}
