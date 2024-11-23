// Gursimran Kainth 

#include <edm4eic/EDM4eicVersion.h>
#include <Evaluator/DD4hepUnits.h>
#include <JANA/JApplication.h>
#include <math.h>
#include <string>

#include "algorithms/calorimetry/ModDbscanClusterConfig.h"
#include "extensions/jana/JOmniFactoryGeneratorT.h"
#include "factories/calorimetry/CalorimeterHitDigi_factory.h"
#include "factories/calorimetry/CalorimeterHitReco_factory.h"
#include "factories/calorimetry/HEXPLIT_factory.h"
#include "factories/calorimetry/ImagingTopoCluster_factory.h"
#include "factories/calorimetry/ModDbscanCluster_factory.h"

extern "C" {
    void InitPlugin(JApplication *app) {

        using namespace eicrecon;

        InitJANAPlugin(app);

        // LYSO part of the ZDC
        app->Add(new JOmniFactoryGeneratorT<CalorimeterHitDigi_factory>(
          "EcalFarForwardZDCRawHits",
          {"EcalFarForwardZDCHits"},
#if EDM4EIC_VERSION_MAJOR >= 7
          {"EcalFarForwardZDCRawHits", "EcalFarForwardZDCRawHitAssociations"},
#else
          {"EcalFarForwardZDCRawHits"},
#endif
          {
            .tRes = 0.0 * dd4hep::ns,
            .capADC = 32768,
            .dyRangeADC = 2000 * dd4hep::MeV,
            .pedMeanADC = 400,
            .pedSigmaADC = 3.2,
            .resolutionTDC = 10 * dd4hep::picosecond,
            .corrMeanScale = "1.0",
            .readout = "EcalFarForwardZDCHits",
          },
          app   // TODO: Remove me once fixed
        ));
        app->Add(new JOmniFactoryGeneratorT<CalorimeterHitReco_factory>(
          "EcalFarForwardZDCRecHits", {"EcalFarForwardZDCRawHits"}, {"EcalFarForwardZDCRecHits"},
          {
            .capADC = 32768,
            .dyRangeADC = 2000. * dd4hep::MeV,
            .pedMeanADC = 400,
            .pedSigmaADC = 3.2,
            .resolutionTDC = 10 * dd4hep::picosecond,
            .thresholdFactor = 4.0,
            .thresholdValue = 0.0,
            .sampFrac = "1.0",
            .readout = "EcalFarForwardZDCHits",
          },
          app   // TODO: Remove me once fixed
        ));

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

        app->Add(new JOmniFactoryGeneratorT<HEXPLIT_factory>(
          "HcalFarForwardZDCSubcellHits", {"HcalFarForwardZDCRecHits"}, {"HcalFarForwardZDCSubcellHits"},
          {
            .MIP = 472. * dd4hep::keV,
            .Emin_in_MIPs=0.5,
            .tmax=269 * dd4hep::ns,
          },
          app   // TODO: Remove me once fixed
        ));

        app->Add(new JOmniFactoryGeneratorT<ModDbscanCluster_factory>(
        "HcalFarForwardZDCModDbscanClusters", 
        {"HcalFarForwardZDCSubcellHits"}, 
        {"HcalFarForwardZDCModDbscanClusters"},
        {
            .epsilon = eicrecon::ModDbscanClusterConfig::epsilon1,            // Use epsilon1 from the config
            .minNeighbors = eicrecon::ModDbscanClusterConfig::minNeighbors1,   // Use minNeighbors1 from the config
            .energyThreshold = eicrecon::ModDbscanClusterConfig::energyThreshold,  // Use energyThreshold from the config
            
        },
        app   // TODO: Remove me once fixed
        ));

    }
}
