// Gursimran Kainth 

#pragma once

#include <string>
#include <iostream>

namespace eicrecon {

  struct ModDbscanClusterConfig {

    // Epsilon value for DBSCAN clustering in mm
    int epsilon1 = 60.0; // Default epsilon value is 60 mm

    // Minimum number of neighbors required for DBSCAN clustering
    int minNeighbors1 = 5;  // Default number of neighbors

    // Energy threshold for DBSCAN clustering
    double energyThreshold = 0.1; // Default energy threshold is 0.1

    // Add any other parameters as needed for DBSCAN

  };

  // Stream operator to read ModDbscanClusterConfig (if applicable)
  std::istream& operator>>(std::istream& in, ModDbscanClusterConfig& config) {
    in >> config.epsilon1 >> config.minNeighbors1 >> config.energyThreshold;
    return in;
  }

  std::ostream& operator<<(std::ostream& out, const ModDbscanClusterConfig& config) {
    out << config.epsilon1 << " " << config.minNeighbors1 << " " << config.energyThreshold;
    return out;
  }

} // namespace eicrecon
