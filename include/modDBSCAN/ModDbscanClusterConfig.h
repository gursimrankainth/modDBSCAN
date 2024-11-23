// Gursimran Kainth 

#pragma once

#include <string>
#include <iostream>

namespace eicrecon {
    struct ModDbscanClusterConfig {

        // epsilon: radius of neighborhood 
        // minNeighbors: minimum number of points required for a dense region
        // energyThreshold: minimum energy required for point to be considered a core point 
        // distanceThreshold: points assigned to a cluster need to be within this distance of one another

        // Step 1: 
        int epsilon1 = 60;
        int minNeighbors1 = 3; 
        double energyThreshold = 0.01; 

        // Optional Step 2 (commented out if not used):
        //int epsilon2 = 50; 
        //int minNeighbors2 = 4; 
        //int distanceThreshold = 100; 

    };

}