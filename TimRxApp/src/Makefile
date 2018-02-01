TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC += TimRxSupport
TimRxSupport_SRCS += drvTimRx.cpp
TimRxSupport_LIBS += asyn
TimRxSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = TimRx
# TimRx.dbd will be created and installed
DBD += $(PROD_IOC).dbd
$(PROD_IOC)_DBD += TimRxSupport.dbd
DBD += TimRxSupport.dbd

# Include dbd files from all support applications:
TimRx_DBD += base.dbd
TimRx_DBD += asyn.dbd

# TimRx_registerRecordDeviceDriver.cpp derives from TimRx.dbd
TimRx_SRCS += TimRx_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
TimRx_SRCS_DEFAULT += TimRxMain.cpp
TimRx_SRCS_vxWorks += -nil-

# Add all the support libraries needed by this IOC
TimRx_LIBS += TimRxSupport
TimRx_LIBS += asyn
TimRx_LIBS += autosave

# Link to the EPICS Base libraries
TimRx_LIBS += $(EPICS_BASE_IOC_LIBS)

# Link to system libraries
TimRx_SYS_LIBS += halcsclient
TimRx_SYS_LIBS += errhand
TimRx_SYS_LIBS += hutils
TimRx_SYS_LIBS += mlm
TimRx_SYS_LIBS += czmq
TimRx_SYS_LIBS += zmq
# FIXME: Why does epics does not include these libs?
TimRx_SYS_LIBS += pthread
TimRx_SYS_LIBS += readline
TimRx_SYS_LIBS += m
TimRx_SYS_LIBS += rt
TimRx_SYS_LIBS += dl
TimRx_SYS_LIBS += gcc

# System header files
TimRx_INCLUDES += -I/usr/include

# Compile test program
PROD += TimRxCheckInit
TimRxCheckInit_SRCS += TimRxCheckInit.c

# Link to system libraries
TimRxCheckInit_SYS_LIBS += halcsclient
TimRxCheckInit_SYS_LIBS += errhand
TimRxCheckInit_SYS_LIBS += hutils
TimRxCheckInit_SYS_LIBS += mlm
TimRxCheckInit_SYS_LIBS += czmq
TimRxCheckInit_SYS_LIBS += zmq
# FIXME: Why does epics does not include these libs?
TimRxCheckInit_SYS_LIBS += pthread
TimRxCheckInit_SYS_LIBS += readline
TimRxCheckInit_SYS_LIBS += m
TimRxCheckInit_SYS_LIBS += rt
TimRxCheckInit_SYS_LIBS += dl
TimRxCheckInit_SYS_LIBS += gcc

# System header files and "any" implementation.
USR_CXXFLAGS += -I/usr/include -I$(TOP)/foreign/any

# CXX Compiler flags
USR_CXXFLAGS += -std=gnu++11 -DMLM_BUILD_DRAFT_API -D__BOARD_AFCV3_1__
USR_CFLAGS += -DMLM_BUILD_DRAFT_API -D__BOARD_AFCV3_1__

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

