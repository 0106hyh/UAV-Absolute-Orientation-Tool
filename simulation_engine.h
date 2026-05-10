#ifndef SIMULATION_ENGINE_H
#define SIMULATION_ENGINE_H

#include "config_reader.h"
#include <string>

class SimulationEngine {
public:
    // Generate simulation data (gps_trajectory.csv and images.txt)
    // based on the parameters in config.sim
    static void generateDataset(const AppConfig& config);
};

#endif // SIMULATION_ENGINE_H
