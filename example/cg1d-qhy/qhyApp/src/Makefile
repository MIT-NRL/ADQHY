TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC application

PROD_IOC = qhy
# qhy.dbd will be created and installed
DBD += qhy.dbd

USR_LDFLAGS += -L/usr/local/lib

# qhy.dbd will be made up from these files:
qhy_DBD += base.dbd
qhy_DBD += ADSupport.dbd
qhy_DBD += NDPluginSupport.dbd
qhy_DBD += NDFileTIFF.dbd
qhy_DBD += QHYSupport.dbd
qhy_DBD += calcSupport.dbd
qhy_DBD += sscanSupport.dbd
qhy_DBD += busySupport.dbd
qhy_DBD += asyn.dbd
qhy_DBD += asSupport.dbd
qhy_DBD += devIocStats.dbd

# qhy_registerRecordDeviceDriver.cpp derives from qhy.dbd
qhy_SRCS += qhy_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
qhy_SRCS_DEFAULT += qhyMain.cpp
qhy_SRCS_vxWorks += -nil-

# Link in the code from the support library
include $(ADCORE)/ADApp/commonDriverMakefile
qhy_LIBS += QHYSupport
qhy_LIBS += devIocStats
qhy_LIBS += autosave
#qhy_LIBS += libqhyccd
SYS_PROD_LIBS += qhyccd

# Finally link to the EPICS Base libraries
qhy_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

