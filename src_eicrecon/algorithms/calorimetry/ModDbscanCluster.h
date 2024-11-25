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

    class DBSCAN_S1 {
    public:
        enum ClusterLabel { UNDEFINED = -2, NOISE = -1 };

        DBSCAN_S1(double epsilon, int min_neighbors, double energy_threshold)
            : epsilon(epsilon), min_neighbors(min_neighbors), energy_threshold(energy_threshold) {}

        std::unordered_map<int, std::vector<std::tuple<float, float, float, float>>> cluster(
            const std::vector<std::tuple<float, float, float, float>>& points) {
            std::vector<int> point_clusters(points.size(), UNDEFINED);
            int current_cluster = 0;

            for (size_t i = 0; i < points.size(); ++i) {
                if (point_clusters[i] != UNDEFINED || is_noise(points[i])) {
                    continue;
                }

                std::vector<size_t> neighbors = range_query(points, i);

                if (neighbors.size() < min_neighbors || !energy_check(points, neighbors)) {
                    point_clusters[i] = NOISE;
                    continue;
                }

                point_clusters[i] = current_cluster;
                expand_cluster(points, point_clusters, neighbors, current_cluster);
                ++current_cluster;
            }

            return split_by_cluster(points, point_clusters);
        }

    private:
        double epsilon;
        int min_neighbors;
        double energy_threshold;

        bool is_noise(const std::tuple<float, float, float, float>& point) const {
            return std::get<2>(point) > 36500 && std::get<3>(point) < energy_threshold;
        }

        std::vector<size_t> range_query(const std::vector<std::tuple<float, float, float, float>>& points, size_t idx) const {
            std::vector<size_t> neighbors;
            const auto& target_point = points[idx];

            for (size_t i = 0; i < points.size(); ++i) {
                if (i != idx && distance(target_point, points[i]) <= epsilon) {
                    neighbors.push_back(i);
                }
            }

            return neighbors;
        }

        double distance(const std::tuple<float, float, float, float>& p1, const std::tuple<float, float, float, float>& p2) const {
            double dx = std::get<0>(p1) - std::get<0>(p2);
            double dy = std::get<1>(p1) - std::get<1>(p2);
            double dz = std::get<2>(p1) - std::get<2>(p2);
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        bool energy_check(const std::vector<std::tuple<float, float, float, float>>& points, const std::vector<size_t>& neighbors) const {
            return std::any_of(neighbors.begin(), neighbors.end(), [&](size_t idx) {
                return std::get<3>(points[idx]) > energy_threshold;
            });
        }

        void expand_cluster(const std::vector<std::tuple<float, float, float, float>>& points, 
                            std::vector<int>& point_clusters,
                            const std::vector<size_t>& seeds, 
                            int current_cluster) {
            std::vector<size_t> to_expand = seeds;
            while (!to_expand.empty()) {
                size_t q_idx = to_expand.back();
                to_expand.pop_back();

                if (point_clusters[q_idx] == NOISE || point_clusters[q_idx] == UNDEFINED) {
                    point_clusters[q_idx] = current_cluster;
                    auto neighbors = range_query(points, q_idx);
                    if (neighbors.size() >= min_neighbors) {
                        to_expand.insert(to_expand.end(), neighbors.begin(), neighbors.end());
                    }
                }
            }
        }

        std::unordered_map<int, std::vector<std::tuple<float, float, float, float>>> split_by_cluster(
            const std::vector<std::tuple<float, float, float, float>>& points,
            const std::vector<int>& point_clusters) const {

            std::unordered_map<int, std::vector<std::tuple<float, float, float, float>>> clusters;

            for (size_t i = 0; i < points.size(); ++i) {
                if (point_clusters[i] != NOISE) {
                    clusters[point_clusters[i]].push_back(points[i]);
                }
            }

            return clusters;
        }
    };

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
        void process(const Input& input, const Output& output) const final {
            const auto [hits] = input;  // Get the input hits
            auto [proto_clusters] = output;  // Get the output clusters

            // Extract position (x, y, z) and energy from the hits
            std::vector<std::tuple<float, float, float, float>> hit_points;
            std::vector<edm4eic::CalorimeterHit> hit_refs;  // Store original hits for reference

            // Loop over hits, collecting (x, y, z, energy) info
            for (std::size_t i = 0; i < hits->size(); ++i) {
                const auto& hit = (*hits)[i];
                auto position = hit.getPosition();
                float x = position.x;
                float y = position.y;
                float z = position.z;
                float energy = hit.getEnergy();

                hit_points.emplace_back(x, y, z, energy);
                hit_refs.push_back(hit);

                debug("Hit {}: position = ({}, {}, {}), energy = {}", i, x, y, z, energy);
            }

            // Instantiate DBSCAN and perform clustering
            DBSCAN_S1 dbscan(epsilon1, minNeighbors1, energyThreshold);
            auto clusters = dbscan.cluster(hit_points);  // Run clustering

            // Create and store the clusters in output (proto_clusters)
            for (const auto& [cluster_id, points] : clusters) {
                if (cluster_id == DBSCAN_S1::NOISE) {
                    continue;  // Skip noise points
                }

                auto pcl = proto_clusters->create();  // Create a new ProtoCluster

                for (const auto& point : points) {
                    // Locate the original hit corresponding to this point
                    auto it = std::find_if(hit_refs.begin(), hit_refs.end(), [&](const edm4eic::CalorimeterHit& hit) {
                        auto pos = hit.getPosition();
                        return pos.x == std::get<0>(point) && pos.y == std::get<1>(point) && pos.z == std::get<2>(point)
                            && hit.getEnergy() == std::get<3>(point);
                    });

                    if (it != hit_refs.end()) {
                        pcl.addToHits(*it);  // Add the hit to the ProtoCluster
                        pcl.addToWeights(1); // Add weight (can adjust as needed)
                    }
                }

                proto_clusters->push_back(pcl);  // Add the constructed ProtoCluster to the output collection
            }
        }

    };

}  // namespace eicrecon
