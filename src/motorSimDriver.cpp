#include "motorSimDriver.hpp"
#include <iocsh.h>
#include <epicsExport.h>
#include <stdio.h>
#include <string.h>
#include <motorRecord.h>
#include <asynMotorController.h>

MotorSimAxis::MotorSimAxis(MotorSimController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC), position_(0.0), velocity_(0.0) {
    asynStatus status;
    status = setDoubleParam(pC_->valIndex, position_);
    if (status != asynSuccess) printf("Axis %d: Failed to set VAL, status=%d\n", axisNo_, status);
    status = setDoubleParam(pC_->veloIndex, velocity_);
    if (status != asynSuccess) printf("Axis %d: Failed to set VELO, status=%d\n", axisNo_, status);
    status = setDoubleParam(pC_->rbvIndex, position_);
    if (status != asynSuccess) printf("Axis %d: Failed to set RBV in constructor, status=%d\n", axisNo_, status);
}

asynStatus MotorSimAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    if (relative) {
        position_ += position;
    } else {
        position_ = position;
    }
    velocity_ = maxVelocity;
    printf("Axis %d moved to position %.2f with velocity %.2f\n", axisNo_, position_, velocity_);
    setDoubleParam(pC_->valIndex, position_);
    setDoubleParam(pC_->veloIndex, velocity_);
    setDoubleParam(pC_->rbvIndex, position_);
    callParamCallbacks();
    return asynSuccess;
}

asynStatus MotorSimAxis::poll(bool *moving) {
    *moving = (velocity_ != 0.0);
    asynStatus status = setDoubleParam(pC_->rbvIndex, position_);
    if (status != asynSuccess) {
        printf("Axis %d: Failed to set RBV in poll, status=%d\n", axisNo_, status);
    } else {
        callParamCallbacks();
    }
    return asynSuccess;
}

asynStatus MotorSimAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration) {
    velocity_ = maxVelocity;
    printf("Axis %d moving at velocity %.2f\n", axisNo_, velocity_);
    setDoubleParam(pC_->veloIndex, velocity_);
    callParamCallbacks();
    return asynSuccess;
}

asynStatus MotorSimAxis::stop(double acceleration) {
    velocity_ = 0.0;
    printf("Axis %d stopped\n", axisNo_);
    setDoubleParam(pC_->veloIndex, velocity_);
    callParamCallbacks();
    return asynSuccess;
}

asynStatus MotorSimAxis::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;

    if (function == pC_->valIndex) {
        position_ = value;
        printf("Axis %d: Set position to %.2f via Float64\n", axisNo_, position_);
        setDoubleParam(pC_->valIndex, position_);
        setDoubleParam(pC_->rbvIndex, position_);
    } else if (function == pC_->veloIndex) {
        velocity_ = value;
        printf("Axis %d: Set velocity to %.2f via Float64\n", axisNo_, velocity_);
        setDoubleParam(pC_->veloIndex, velocity_);
    } else {
        status = asynError;
        printf("Axis %d: Unknown parameter %d\n", axisNo_, function);
    }
    callParamCallbacks();
    return status;
}

MotorSimController::MotorSimController(const char *portName)
    : asynMotorController(portName, NUM_AXES, NUM_MOTOR_PARAMS,
                          0, 0,                    // interfaceMask, interruptMask
                          ASYN_MULTIDEVICE | ASYN_CANBLOCK,  // asynFlags
                          1, 0, 0) {             // autoConnect, priority, stackSize
    createParam("VAL", asynParamFloat64, &valIndex);
    createParam("VELO", asynParamFloat64, &veloIndex);
    createParam("RBV", asynParamFloat64, &rbvIndex);

    for (int i = 0; i < NUM_AXES; i++) {
        new MotorSimAxis(this, i);
    }
    printf("Motor simulator configured with %d axes on port %s\n", NUM_AXES, portName);
    startPoller(0.1, 0.5, 2);
}

asynStatus MotorSimController::poll() {
    for (int i = 0; i < NUM_AXES; i++) {
        bool moving;
        MotorSimAxis *axis = getAxis(i);
        if (axis) axis->poll(&moving);
    }
    return asynSuccess;
}

asynStatus MotorSimController::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize) {
    if (strcmp(drvInfo, "MOTOR") == 0) {
        pasynUser->reason = valIndex;
        if (pptypeName) *pptypeName = "Float64";
        if (psize) *psize = sizeof(epicsFloat64);
        return asynSuccess;
    }
    return asynMotorController::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
}

extern "C" {
    int motorSimConfigure(const char *portName) {
        new MotorSimController(portName);
        return 0;
    }

    static const iocshArg configArg0 = {"portName", iocshArgString};
    static const iocshArg *const configArgs[] = {&configArg0};
    static const iocshFuncDef configFuncDef = {"motorSimConfigure", 1, configArgs};

    static void configCallFunc(const iocshArgBuf *args) {
        motorSimConfigure(args[0].sval);
    }

    static void motorSimRegister(void) {
        iocshRegister(&configFuncDef, configCallFunc);
    }

    epicsExportRegistrar(motorSimRegister);
}
