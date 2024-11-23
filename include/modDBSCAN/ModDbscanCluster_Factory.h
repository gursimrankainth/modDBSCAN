// Gursimran Kainth 

#pragma once

#include "algorithms/calorimetry/ModDbscanCluster_Factory.h"
#include "extensions/jana/JOmniFactory.h"
#include "services/algorithms_init/AlgorithmsInit_service.h"

namespace eicrecon {

    class ModDbscanCluster_Factory : public JOmniFactory<ModDbscanCluster_Factory, ModDbscanClusterConfig> {

    public: 
        using AlgoDBSCAN = eicrecon::ModDbscanCluster;  // Alias for the algorithm

    private:
        std::unique_ptr<AlgoDBSCAN> m_algo;  // Pointer to the algorithm instance

        PodioInput<edm4eic::CalorimeterHitCollection> m_hits_input {this};  // Input collection of hits
        PodioOutput<edm4eic::ProtoClusterCollection> m_ProtoCluster_output {this};  // Output collection for ProtoClusters

        // Parameters for clustering: epsilon, min_neighbors, energy_threshold
        ParameterRef<int> m_eps_1 {this, "epsilon1", config().epsilon1};  // Epsilon value for DBSCAN
        ParameterRef<int> m_minNbrs_1 {this, "minNeighbors1", config().minNeighbors1};  // Minimum number of neighbors for DBSCAN
        ParameterRef<double> m_eThres {this, "energyThreshold", config().energyThreshold};  // Energy threshold for DBSCAN

        Service<AlgorithmsInit_service> m_algorithmsInit {this};  // Service for algorithm initialization

    public:
        // Configure the factory and set up the algorithm
        void Configure() {
            m_algo = std::make_unique<AlgoDBSCAN>(GetPrefix());  // Initialize algorithm instance with prefix
            m_algo->level(static_cast<algorithms::LogLevel>(logger()->level()));  // Set logging level
            m_algo->applyConfig(config());  // Apply configuration from ModDbscanClusterConfig
            m_algo->init();  // Initialize the algorithm
        }

        // Handle changes in the run number, if necessary
        void ChangeRun(int64_t run_number) {
            // Placeholder for any run-specific logic if needed
        }

        // Process the event and apply the DBSCAN clustering
        void Process(int64_t run_number, uint64_t event_number) {
            // Pass the input hits and the ProtoCluster output to the algorithm
            m_algo->process({m_hits_input()}, {m_ProtoCluster_output().get()});  
        }
    };

}



