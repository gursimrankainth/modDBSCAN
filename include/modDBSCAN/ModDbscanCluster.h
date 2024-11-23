// Gursimran Kainth 

#pragma once

#include <algorithm>
#include <numeric>

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
        void process(const Input& input, const Output& output) const final {
            const auto [hits] = input;  // Get the input hits
            auto [proto_clusters] = output;  // Get the output clusters

            // Extract position (x, y, z) and energy from the hits
            std::vector<std::tuple<float, float, float, float>> hit_points;

            // Loop over hits, collecting (x, y, z, energy) info
            for (std::size_t i = 0; i < hits->size(); ++i) {
                auto hit = (*hits)[i];
                auto position = hit.getPosition();
                float x = position.x;
                float y = position.y;
                float z = position.z;
                float energy = hit.getEnergy();

                hit_points.push_back(std::make_tuple(x, y, z, energy));
            }

            // Perform DBSCAN clustering on hit_points
            DBSCAN_S1 dbscan(epsilon1, minNeighbors1, energyThreshold);  // Assuming DBSCAN_S1 is your DBSCAN class
            auto clusters = dbscan.cluster(hit_points);  // Run clustering

            // Create and store the clusters in output (proto_clusters)
            for (const auto& [cluster_id, points] : clusters) {
                edm4eic::ProtoCluster new_cluster;

                for (const auto& point : points) {
                    edm4eic::CalorimeterHit hit;
                    hit.setPosition({std::get<0>(point), std::get<1>(point), std::get<2>(point)});
                    hit.setEnergy(std::get<3>(point));
                    new_cluster.addToHits(hit.getObjectID());
                }

                proto_clusters->push_back(new_cluster);  // Add the constructed cluster to the output collection
            }
        }
    };

    class DBSCAN_S1 {
    public:
        enum ClusterLabel { UNDEFINED = -2, NOISE = -1 };

        DBSCAN_S1(double epsilon, int min_neighbors, double energy_threshold)
            : epsilon(epsilon), min_neighbors(min_neighbors), energy_threshold(energy_threshold) {}

        std::unordered_map<int, std::vector<std::tuple<float, float, float, float>>> cluster(
            const std::vector<std::tuple<float, float, float, float>>& points) {
            std::vector<int> point_clusters(points.size(), UNDEFINED);  // Initialize all points as undefined
            int current_cluster = 0;

            for (size_t i = 0; i < points.size(); ++i) {
                if (point_clusters[i] != UNDEFINED) {
                    continue;
                }

                if (is_noise(points[i])) {
                    point_clusters[i] = NOISE;
                    continue;
                }

                std::vector<size_t> neighbors = range_query(points, i);

                if (neighbors.size() < min_neighbors || !energy_check(points, neighbors)) {
                    point_clusters[i] = NOISE;
                    continue;
                }

                point_clusters[i] = current_cluster;
                std::vector<size_t> seeds = neighbors;

                while (!seeds.empty()) {
                    size_t q_point_idx = seeds.back();
                    seeds.pop_back();

                    if (point_clusters[q_point_idx] == NOISE) {
                        point_clusters[q_point_idx] = current_cluster;
                    } else if (point_clusters[q_point_idx] != UNDEFINED) {
                        continue;
                    }

                    point_clusters[q_point_idx] = current_cluster;
                    std::vector<size_t> q_neighbors = range_query(points, q_point_idx);

                    if (q_neighbors.size() >= min_neighbors) {
                        seeds.insert(seeds.end(), q_neighbors.begin(), q_neighbors.end());
                    }
                }

                ++current_cluster;
            }

            return split_by_cluster(points, point_clusters);
        }

    private:
        double epsilon;
        int min_neighbors;
        double energy_threshold;

        bool is_noise(const std::tuple<float, float, float, float>& point) const {
            return std::get<2>(point) > 36500 && std::get<3>(point) < energy_threshold;  // Check z position and energy
        }

        std::vector<size_t> range_query(const std::vector<std::tuple<float, float, float, float>>& points, size_t idx) const {
            std::vector<size_t> neighbors;
            const auto& target_point = points[idx];

            for (size_t i = 0; i < points.size(); ++i) {
                if (i == idx) continue;
                if (distance(target_point, points[i]) <= epsilon) {
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
            for (size_t idx : neighbors) {
                if (std::get<3>(points[idx]) > energy_threshold) {
                    return true;
                }
            }
            return false;
        }

        std::unordered_map<int, std::vector<std::tuple<float, float, float, float>>> split_by_cluster(
            const std::vector<std::tuple<float, float, float, float>>& points,
            const std::vector<int>& point_clusters) const {

            std::unordered_map<int, std::vector<std::tuple<float, float, float, float>>> clusters;

            for (size_t i = 0; i < points.size(); ++i) {
                int cluster_id = point_clusters[i];
                if (cluster_id != NOISE) {
                    clusters[cluster_id].push_back(points[i]);
                }
            }

            return clusters;
        }
    };

}  // namespace eicrecon
