TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library

USR_CXXFLAGS += -Wunused-but-set-variable -std=c++0x

#LIBUSB1 is set in configure/CONFIG_SITE
USR_INCLUDES += -I$(LIBUSB1)
#USR_INCLUDES += -I$(QHY_VENDOR)

LIBRARY_IOC += QHY

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# Install devXxxSoft.dbd into <top>/dbd
DBD += QHYSupport.dbd

# Compile and add the code to the support library
LIB_SRCS += QHY.cpp

LIB_SYS_LIBS_Linux += qhyccd

# These are compiled as part of the top level Make,
# before we get to compiling the support module.
#USR_OBJS += $(QHY_VENDOR)/csbigcam.o
#USR_OBJS += $(QHY_VENDOR)/csbigimg.o
#USR_OBJS += libqhyccd.so

# QHY_LIBS += $(EPICS_BASE_IOC_LIBS)


include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

