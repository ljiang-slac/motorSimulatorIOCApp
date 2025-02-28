TOP=..

include $(TOP)/configure/CONFIG

PROD_IOC = MotorSimulator
DBD += MotorSimulator.dbd

MOTOR_DBD = /epics/motor/dbd
ASYN_DBD = /epics/asyn/dbd
MOTOR_INCLUDE = /epics/motor/include
ASYN_INCLUDE = /epics/asyn/include

MotorSimulator_DBD += base.dbd
MotorSimulator_DBD += $(MOTOR_DBD)/motorSupport.dbd
MotorSimulator_DBD += $(ASYN_DBD)/asyn.dbd
MotorSimulator_DBD += motorSim.dbd  #

USR_DBDFLAGS += -I $(MOTOR_DBD)
USR_DBDFLAGS += -I $(ASYN_DBD)

USR_INCLUDES += -I $(MOTOR_INCLUDE)
USR_INCLUDES += -I $(ASYN_INCLUDE)

MotorSimulator_LIBS += motor
MotorSimulator_LIBS += asyn
MotorSimulator_LIBS += $(EPICS_BASE_IOC_LIBS)

MotorSimulator_SRCS += MotorSimulator_registerRecordDeviceDriver.cpp
MotorSimulator_SRCS += motorSimDriver.cpp
MotorSimulator_SRCS_DEFAULT += MotorSimulatorMain.cpp
MotorSimulator_SRCS_vxWorks += -nil-

motor_DIR = /epics/motor/lib/linux-x86_64
asyn_DIR = /epics/asyn/lib/linux-x86_64
EPICS_BASE_LIB_DIR = /epics/epics-base/lib/linux-x86_64

USR_CXXFLAGS += -std=c++11

include $(TOP)/configure/RULES
