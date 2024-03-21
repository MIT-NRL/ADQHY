#!../../bin/linux-x86_64/qhy

## You may have to change qhy to something else
## everywhere it appears in this file

< envPaths

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

cd ${TOP}

epicsEnvSet("QHY_PV", "CG1D:Det:Q1")

## Register all support components
dbLoadDatabase "dbd/qhy.dbd"
qhy_registerRecordDeviceDriver pdbbase

#################################################
# Set up the QHY driver

QHYConfig("Q1.CAM",-1,-1)

#################################################
# Set up the areaDetector plugins

NDTransformConfigure("Q1.TRANQ1", 10, 0, "Q1.CAM", 0, -1, -1, 0, 0)
NDROIConfigure("Q1.ROI1", 10, 0, "Q1.TRANQ1", 0, -1, -1, 0, 0)
NDStdArraysConfigure("Q1.ARR1", 10, 0, "Q1.ROI1", 0, -1, 0, 0)
NDStatsConfigure("Q1.STATQ1", 10, 0, "Q1.TRANQ1", 0, -1, -1, 0, 0)
NDFileTIFFConfigure("Q1.TIFF1", 10, 0, "Q1.TRANQ1", 0, 0, 0, 0)

#################################################
# autosave

epicsEnvSet IOCNAME qhy
epicsEnvSet SAVE_DIR /home/controls/var/cg1d-qhy

save_restoreSet_Debug(0)

### status-PV prefix, so save_restore can find its status PV's.
save_restoreSet_status_prefix("CG1D:CS:Q1:")

set_requestfile_path("$(SAVE_DIR)")
set_savefile_path("$(SAVE_DIR)")

save_restoreSet_NumSeqFiles(3)
save_restoreSet_SeqPeriodInSeconds(600)
set_pass0_restoreFile("$(IOCNAME).sav")
set_pass1_restoreFile("$(IOCNAME).sav")

#################################################

## Load record instances
dbLoadRecords "db/qhy.db"

# Access Security
asSetFilename("$(TOP)/../cg1d-AccessSecurity/db/cg1d.acf")
asSetSubstitutions("P=CG1D:CS")

cd ${TOP}/iocBoot/${IOC}
iocInit

# Create request file and start periodic 'save'
epicsThreadSleep(5)
# Note: the .req file has to be manually copied from the IOC directory
# to the autosave directory, because we don't autogenerate it for this IOC.
create_monitor_set("$(IOCNAME).req", 10)

# Enable plugins at startup (settings that are not autosaved)
dbpf $(QHY_PV):ArrayCallbacks 1
dbpf $(QHY_PV):Trans1:EnableCallbacks 1
dbpf $(QHY_PV):TIFF1:EnableCallbacks 1
dbpf $(QHY_PV):Stats1:EnableCallbacks 1
dbpf $(QHY_PV):ROI1:EnableCallbacks 1
dbpf $(QHY_PV):Array1:EnableCallbacks 1

# Set up ROI binning by default for the array plugin
# We also enable scaling (divide by 4 because we are doing 2x2 binning).
# We need to force the output data type to be UInt16, otherwise the plugin 
# will convert to floating point.
dbpf $(QHY_PV):ROI1:BinX 8
dbpf $(QHY_PV):ROI1:BinY 8
dbpf $(QHY_PV):ROI1:DataTypeOut 3
dbpf $(QHY_PV):ROI1:EnableScale 1
dbpf $(QHY_PV):ROI1:Scale 64

# Set up TIFF plugin auto increment
dbpf $(QHY_PV):TIFF1:AutoIncrement 1
dbpf $(QHY_PV):TIFF1:AutoSave 1

# Set the NDAttributesFile paths
dbpf $(QHY_PV):NDAttributesFile "/home/controls/cg1d/NDAttributeFiles/cg1d-qhy600.xml"
dbpf $(QHY_PV):TIFF1:NDAttributesFile "/home/controls/cg1d/NDAttributeFiles/cg1d-file.xml"



