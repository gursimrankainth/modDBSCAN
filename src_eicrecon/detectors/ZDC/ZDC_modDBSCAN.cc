// Gursimran Kainth 

#include <edm4eic/EDM4eicVersion.h>
#include <Evaluator/DD4hepUnits.h>
#include <JANA/JApplication.h>
#include <math.h>
#include <string>

#include "algorithms/calorimetry/ImagingTopoClusterConfig.h"
#include "extensions/jana/JOmniFactoryGeneratorT.h"
#include "factories/calorimetry/CalorimeterHitDigi_factory.h"
#include "factories/calorimetry/CalorimeterHitReco_factory.h"
#include "factories/calorimetry/HEXPLIT_factory.h"
#include "factories/calorimetry/ModDbscanCluster_factory.h"

extern "C" {
    void InitPlugin(JApplication *app) {

        using namespace eicrecon;

        InitJANAPlugin(app);

        // 1. Adding Hcal-based clusters, similar to Ecal
        app->Add(new JOmniFactoryGeneratorT<CalorimeterHitDigi_factory>(
          "HcalFarForwardZDCRawHits",
          {"HcalFarForwardZDCHits"},
#if EDM4EIC_VERSION_MAJOR >= 7
          {"HcalFarForwardZDCRawHits", "HcalFarForwardZDCRawHitAssociations"},
#else
          {"HcalFarForwardZDCRawHits"},
#endif
          {
            .tRes = 0.0 * dd4hep::ns,
            .capADC = 65536,
            .dyRangeADC = 1000. * dd4hep::MeV,
            .pedMeanADC = 400,
            .pedSigmaADC = 2,
            .resolutionTDC = 10 * dd4hep::picosecond,
            .corrMeanScale = "1.0",
            .readout = "HcalFarForwardZDCHits",
          },
          app   // TODO: Remove me once fixed
        ));

        // 2. Adding CalorimeterHitReco_factory
        app->Add(new JOmniFactoryGeneratorT<CalorimeterHitReco_factory>(
          "HcalFarForwardZDCRecHits", {"HcalFarForwardZDCRawHits"}, {"HcalFarForwardZDCRecHits"},
          {
            .capADC = 65536,
            .dyRangeADC = 1000. * dd4hep::MeV,
            .pedMeanADC = 400,
            .pedSigmaADC = 2,
            .resolutionTDC = 10 * dd4hep::picosecond,
            .thresholdFactor = 3.0,
            .thresholdValue = 0.0,
            .sampFrac = "1.0",
            .readout = "HcalFarForwardZDCHits",
            .layerField = "layer",
            .sectorField = "system",
          },
          app   // TODO: Remove me once fixed
        ));

        // 3. Adding HEXPLIT factory
        app->Add(new JOmniFactoryGeneratorT<HEXPLIT_factory>(
          "HcalFarForwardZDCSubcellHits", {"HcalFarForwardZDCRecHits"}, {"HcalFarForwardZDCSubcellHits"},
          {
            .MIP = 472. * dd4hep::keV,
            .Emin_in_MIPs=0.5,
            .tmax=269 * dd4hep::ns,
          },
          app   // TODO: Remove me once fixed
        ));

        // 4. Adding the new ModDbscanCluster_factory algorithm
        app->Add(new JOmniFactoryGeneratorT<ModDbscanCluster_factory>(
          "HcalFarForwardZDCImagingProtoClusters",  // Name of the DBSCAN clusters
          {"HcalFarForwardZDCSubcellHits"},  // Input hit collection (same as ImagingTopoCluster's input collection)
          {"HcalFarForwardZDCImagingProtoClusters"},  // Output cluster collection (same as ImagingTopoCluster's output collection)
          {
              .epsilon1 = 60,  // Epsilon1 value for DBSCAN clustering
              .minNeighbors1 = 3,  // Minimum number of neighbors for DBSCAN clustering
              .energyThreshold = 0.1,  // Energy threshold for clustering
          },
          app  // Application pointer
      ));

    }
}


