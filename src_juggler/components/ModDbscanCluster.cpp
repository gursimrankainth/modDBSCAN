// Gursimran Kainth 

#include "fmt/format.h"
#include <algorithm>
#include <cmath>
#include <vector>

#include "Gaudi/Property.h"
#include "GaudiAlg/GaudiAlgorithm.h"
#include "GaudiKernel/ToolHandle.h"

#include "DD4hep/BitFieldCoder.h"

#include <k4FWCore/DataHandle.h>
#include <k4Interface/IGeoSvc.h>
#include "JugReco/ClusterTypes.h"

// Event Model related classes
#include "edm4eic/CalorimeterHitCollection.h"
#include "edm4eic/ProtoClusterCollection.h"

using namespace Gaudi::Units;

namespace Jug::Reco {

/**
 * modDBSCAN Clustering Algorithm.
 *
 * This algorithm implements a modified DBSCAN clustering for calorimeter hits.
 *
 * \ingroup reco
 */
class modDBSCAN : public GaudiAlgorithm {
private:
  // Distance threshold for determining neighbors
  Gaudi::Property<double> m_epsilon{this, "epsilon", 1.0};
  // Minimum number of points to form a cluster
  Gaudi::Property<int> m_minPts{this, "minPts", 3};
  // Energy threshold for clustering
  Gaudi::Property<double> m_energyThreshold{this, "energyThreshold", 0.0};

  // Input hits collection
  DataHandle<edm4eic::CalorimeterHitCollection> m_inputHitCollection{"inputHitCollection", Gaudi::DataHandle::Reader, this};
  // Output clustered hits
  DataHandle<edm4eic::ProtoClusterCollection> m_outputProtoClusterCollection{"outputProtoClusterCollection", Gaudi::DataHandle::Writer, this};

public:
  modDBSCAN(const std::string& name, ISvcLocator* svcLoc) : GaudiAlgorithm(name, svcLoc) {
    declareProperty("inputHitCollection", m_inputHitCollection, "");
    declareProperty("outputProtoClusterCollection", m_outputProtoClusterCollection, "");
  }

  StatusCode initialize() override {
    if (GaudiAlgorithm::initialize().isFailure()) {
      return StatusCode::FAILURE;
    }

    info() << fmt::format("DBSCAN clustering parameters: epsilon = {:.2f}, minPts = {}, energyThreshold = {:.2f}",
                          m_epsilon.value(), m_minPts.value(), m_energyThreshold.value())
           << endmsg;

    return StatusCode::SUCCESS;
  }

  StatusCode execute() override {
    const auto& hits = *m_inputHitCollection.get();
    auto& proto = *m_outputProtoClusterCollection.createAndPut();

    // Clustering process
    std::vector<int> labels(hits.size(), -1); // -1 for noise
    int clusterId = 0;

    for (size_t i = 0; i < hits.size(); ++i) {
      if (labels[i] != -1 || hits[i].getEnergy() < m_energyThreshold) {
        continue;
      }
      if (expandCluster(i, hits, labels, clusterId)) {
        ++clusterId;
      }
    }

    // Convert clusters into ProtoClusters
    for (int cid = 0; cid < clusterId; ++cid) {
      auto pcl = proto.create();
      for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == cid) {
          pcl.addToHits(hits[i]);
          pcl.addToWeights(1); // Arbitrary weight for now
        }
      }
    }

    return StatusCode::SUCCESS;
  }

private:
  // Expand cluster from a given point
  bool expandCluster(size_t idx, const edm4eic::CalorimeterHitCollection& hits, std::vector<int>& labels, int clusterId) {
    auto neighbors = regionQuery(idx, hits);
    if (neighbors.size() < static_cast<size_t>(m_minPts.value())) {
      labels[idx] = -1; // Mark as noise
      return false;
    }

    // Start a new cluster
    labels[idx] = clusterId;
    for (size_t i = 0; i < neighbors.size(); ++i) {
      size_t nIdx = neighbors[i];
      if (labels[nIdx] == -1) {
        labels[nIdx] = clusterId; // Include noise point in cluster
      }
      if (labels[nIdx] != -1) {
        continue;
      }
      labels[nIdx] = clusterId;
      auto secondaryNeighbors = regionQuery(nIdx, hits);
      if (secondaryNeighbors.size() >= static_cast<size_t>(m_minPts.value())) {
        neighbors.insert(neighbors.end(), secondaryNeighbors.begin(), secondaryNeighbors.end());
      }
    }

    return true;
  }

  // Find neighbors within epsilon distance
  std::vector<size_t> regionQuery(size_t idx, const edm4eic::CalorimeterHitCollection& hits) const {
    std::vector<size_t> neighbors;
    for (size_t i = 0; i < hits.size(); ++i) {
      if (distance(hits[idx], hits[i]) <= m_epsilon.value()) {
        neighbors.push_back(i);
      }
    }
    return neighbors;
  }

  // Compute distance between two hits
  double distance(const edm4eic::CalorimeterHit& h1, const edm4eic::CalorimeterHit& h2) const {
    return std::sqrt(std::pow(h1.getPosition().x - h2.getPosition().x, 2) +
                     std::pow(h1.getPosition().y - h2.getPosition().y, 2) +
                     std::pow(h1.getPosition().z - h2.getPosition().z, 2));
  }
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
DECLARE_COMPONENT(modDBSCAN)

} // namespace Jug::Reco
