## modDBSCAN
EICRecon/Juggler script for a modified DBSCAN algorithm 


# Instructions for Installing the new ZDC_modDBSCAN Plugin: 
1. Make a new folder in the EICrecon/src/detectors directory called "ZDC_modDBSCAN". This is where the ZDC_modDBSCAN.cc and CMakeLists.txt scripts will be placed.
2. Add ZDC_modDBSCAN.cc and CMakeLists.txt into the ZDC_modDBSCAN plugin directory
3. Add the following line to the end of the CMakeLists.txt in the src/detectors directory so the new plugin compiles with the full EICrecon build, “add_subdirectory(ZDC_modDBSCAN)”
4. Place the algorithm, and factory scripts in their respective folders within their respective EICrecon/src directories. The structure of the directories is mimicked by this Git repository.
6. Move to the EICrecon/build directory and in command line run, "cmake ..". You need to run this since we modified one of the CMake scripts that the framework is dependent on. 
7. Now run “make”
   
# Run only the ZDC_modDBSCAN Plugin: 
source EICrecon/install/bin/eicrecon-this.sh
source /opt/detector/epic-main/bin/thisepic.sh epic_zdc_lyso_sipm
eicrecon -Ppodio:output_collections=HcalFarForwardZDCHits,HcalFarForwardZDCRawHits,HcalFarForwardZDCRecHits,HcalFarForwardZDCSubcellHits,HcalFarForwardZDCImagingProtoClusters \
  -Pplugins=ZDC_modDBSCAN -Pplugins_to_ignore="ZDC,EEMC,BEMC,FEMC,B0ECAL,BTRK,BVTX,ECTRK,MPGD,B0TRK,RPOTS,FOFFMTRK,BTOF,ECTOF,LOWQ2,LUMISPECCAL,PFRICH,DIRC,DRICH,EHCAL,BHCAL" \
  -Ppodio:output_file=LOCATION AND NAME OF OF OUTPUT FILE \
  LOCATION OF INPUT FILE

# Notes: 
- ZDC and ZDC_modDBSCAN plugins conflict. You need to diable the ZDC plugin as shown in the run command above if you would like to use ZDC_modDBSCAN instead.
- ZDC_modDBSCAN is dependent on the FHCAL plugin. If you disable this it will run into issues. 

