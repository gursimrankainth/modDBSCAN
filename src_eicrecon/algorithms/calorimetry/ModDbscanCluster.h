// Gursimran Kainth 

#pragma once

#include <algorithm>
#include <numeric>
#include <unordered_map>

#include <algorithms/algorithm.h>
#include <DD4hep/BitFieldCoder.h>
#include <DDRec/CellIDPositionConverter.h>
#include <DDRec/Surface.h>
#include <DDRec/SurfaceManager.h>

#include <spdlog/spdlog.h>

// Event model related classes
#include <edm4eic/CalorimeterHitCollection.h>
#include <edm4eic/ProtoClusterCollection.h>
#include <edm4hep/utils/vector_utils.h>

#include "algorithms/interfaces/WithPodConfig.h"
#include "ModDbscanClusterConfig.h"

namespace eicrecon {

    using ModDbscanClusterAlgorithm = algorithms::Algorithm<
        algorithms::Input<edm4eic::CalorimeterHitCollection>,
        algorithms::Output<edm4eic::ProtoClusterCollection>
    >;

    class ModDbscanCluster
        : public ModDbscanClusterAlgorithm,
          public WithPodConfig<ModDbscanClusterConfig> {

    public:
        ModDbscanCluster(std::string_view name)
            : ModDbscanClusterAlgorithm{name,
                {"inputHitCollection"},
                {"outputProtoClusterCollection"},
                "Modified DBSCAN clustering algorithm for ZDC."} {}

    private:
        int epsilon1{0};
        int minNeighbors1{0};
        double energyThreshold{0};

    public:
        void init() {
            // Sanity checks for configuration values
            if (m_cfg.epsilon1 <= 0) {
                error("Epsilon value for DBSCAN must be positive.");
                return;
            }
            if (m_cfg.minNeighbors1 <= 0) {
                error("Minimum number of neighbors must be positive.");
                return;
            }
            if (m_cfg.energyThreshold < 0) {
                error("Energy threshold cannot be negative.");
                return;
            }

            // Assign values from the configuration
            epsilon1 = m_cfg.epsilon1;
            minNeighbors1 = m_cfg.minNeighbors1;
            energyThreshold = m_cfg.energyThreshold;

            // Summarize the clustering parameters
            info("Epsilon: Radius of neighborhood <= [{} mm].", epsilon1);
            info("Minimum Neighbors: Minimum number of hits required to form a cluster <= [{}].", minNeighbors1);
            info("Energy Threshold: Cluster core hit energy threshold <= [{} GeV].", energyThreshold);
        }

        void process(const Input& input, const Output& output) const final {
            const auto [hits] = input;
            auto [proto] = output;

            // Remove duplicate hits
            std::set<std::size_t> indices;
            for (std::size_t i = 0; i < hits->size(); ++i) {
                indices.insert(i);  // Insert all indices (set automatically removes duplicates)
            }

            // Apply DBSCAN algorithm
            info("Starting DBSCAN clustering...");
            std::vector<std::vector<std::size_t>> clusters;  // Store clusters
            std::vector<std::size_t> noise;  // Store noise points

            // Loop over hits to identify core points and expand clusters
            std::vector<bool> visited(hits->size(), false);  // Track visited hits
            std::vector<bool> isCore(hits->size(), false);   // Track core points

            for (std::size_t i = 0; i < hits->size(); ++i) {
                if (!visited[i] && (*hits)[i].getEnergy() >= energyThreshold) {
                    visited[i] = true;
                    // Find neighbors and determine if it's a core point
                    std::vector<std::size_t> indicesVector(indices.begin(), indices.end());  // Convert set to vector
                    std::vector<std::size_t> neighbors = findNeighbors(i, indicesVector, *hits);

                    if (neighbors.size() >= minNeighbors1) {
                        isCore[i] = true;
                        std::vector<std::size_t> cluster;
                        expandCluster(i, indicesVector, *hits, visited, cluster);  // Expand cluster without recursion
                        clusters.push_back(cluster);  // Add the new cluster
                    } else {
                        noise.push_back(i);  // Add to noise if not a core point
                    }
                }
            }

            // Create ProtoClusters from the clusters
            for (const auto& cluster : clusters) {
                auto pcl = proto->create();  // Create a new ProtoCluster
                for (std::size_t idx : cluster) {
                    pcl.addToHits((*hits)[idx]);  // Add hits to the ProtoCluster
                    pcl.addToWeights(1);  // Add weight (default to 1 if no specific weight is provided)
                }
            }

            // Create a single ProtoCluster for all noise hits
            if (!noise.empty()) {
                auto pcl = proto->create();  // Create a new ProtoCluster for noise
                for (std::size_t idx : noise) {
                    pcl.addToHits((*hits)[idx]);  // Add noise hits to the ProtoCluster
                    pcl.addToWeights(1);  // Add weight (default to 1)
                }
            }
        }

    private:
        // Function to find all neighbors of a point within epsilon1 distance
        std::vector<std::size_t> findNeighbors(std::size_t idx, const std::vector<std::size_t>& indices, const edm4eic::CalorimeterHitCollection& hits) const {
            std::vector<std::size_t> neighbors;
            const auto& hit = hits[idx];

            for (const auto& neighborIdx : indices) {
                if (neighborIdx == idx) continue;  // Skip the hit itself
                const auto& neighbor = hits[neighborIdx];
                double dx = hit.getPosition().x - neighbor.getPosition().x;
                double dy = hit.getPosition().y - neighbor.getPosition().y;
                double dz = hit.getPosition().z - neighbor.getPosition().z;
                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                if (distance <= epsilon1) {
                    neighbors.push_back(neighborIdx);  // Add neighbor if within epsilon distance
                }
            }

            return neighbors;
        }

        // Expand a cluster by adding neighboring points (non-recursive)
        void expandCluster(std::size_t idx, const std::vector<std::size_t>& indices, const edm4eic::CalorimeterHitCollection& hits,
                           std::vector<bool>& visited, std::vector<std::size_t>& cluster) const {
            std::vector<std::size_t> toVisit = {idx};  // Start with the current core point

            while (!toVisit.empty()) {
                std::size_t currentIdx = toVisit.back();
                toVisit.pop_back();

                if (!visited[currentIdx]) {
                    visited[currentIdx] = true;
                    cluster.push_back(currentIdx);

                    // Find neighbors and add them to the list to visit
                    std::vector<std::size_t> neighbors = findNeighbors(currentIdx, indices, hits);
                    toVisit.insert(toVisit.end(), neighbors.begin(), neighbors.end());  // Add all neighbors
                }
            }
        }

    };

}  // namespace eicrecon