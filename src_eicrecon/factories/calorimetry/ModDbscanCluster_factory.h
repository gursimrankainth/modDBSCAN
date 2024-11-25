// Gursimran Kainth 

#pragma once

#include "algorithms/calorimetry/ModDbscanCluster.h"
#include "extensions/jana/JOmniFactory.h"
#include "services/algorithms_init/AlgorithmsInit_service.h"

namespace eicrecon {

class ModDbscanCluster_factory : public JOmniFactory<ModDbscanCluster_factory, ModDbscanClusterConfig> {

public:
    using AlgoT = eicrecon::ModDbscanCluster;
private:
    std::unique_ptr<AlgoT> m_algo;

    PodioInput<edm4eic::CalorimeterHit> m_hits_input {this};
    PodioOutput<edm4eic::ProtoCluster> m_protos_output {this};

    // Initialize the DBSCAN parameters from config
    ParameterRef<int> m_eps_1 {this, "epsilon1", config().epsilon1};  // Epsilon value for DBSCAN
    ParameterRef<int> m_minNbrs_1 {this, "minNeighbors1", config().minNeighbors1};  // Min neighbors for DBSCAN
    ParameterRef<double> m_eThres {this, "energyThreshold", config().energyThreshold};  // Energy threshold for DBSCAN

    // Initialize any additional parameters here (if applicable)

    Service<AlgorithmsInit_service> m_algorithmsInit {this};  // Service for algorithm initialization

public:
    void Configure() {
        // Initialize the algorithm
        m_algo = std::make_unique<AlgoT>(GetPrefix());
        m_algo->level(static_cast<algorithms::LogLevel>(logger()->level()));
        m_algo->applyConfig(config());  // Apply config parameters to the algorithm
        m_algo->init();  // Call any additional initialization steps for the algorithm
    }

    void ChangeRun(int64_t run_number) {
        // Handle any changes required at the start of a new run (if needed)
    }

    void Process(int64_t run_number, uint64_t event_number) {
        // Process the input data with the algorithm and write output
        m_algo->process({m_hits_input()}, {m_protos_output().get()});
    }
};

} // namespace eicrecon





