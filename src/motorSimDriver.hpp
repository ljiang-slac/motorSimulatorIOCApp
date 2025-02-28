#ifndef MOTOR_SIM_DRIVER_HPP
#define MOTOR_SIM_DRIVER_HPP

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <motorRecord.h>

#define NUM_AXES 40
#define NUM_MOTOR_PARAMS 256

class MotorSimController;

class MotorSimAxis : public asynMotorAxis {
public:
    MotorSimAxis(MotorSimController *pC, int axisNo);
    virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) override;
    virtual asynStatus poll(bool *moving) override;
    virtual asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration) override;
    virtual asynStatus stop(double acceleration) override;
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);  // 添加接口

private:
    MotorSimController *pC_;
    double position_;
    double velocity_;
};

class MotorSimController : public asynMotorController {
public:
    MotorSimController(const char *portName);
    virtual asynStatus poll() override;
    MotorSimAxis *getAxis(int axisNo) { return static_cast<MotorSimAxis*>(asynMotorController::getAxis(axisNo)); }
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize) override;  // 添加接口

    int valIndex;
    int veloIndex;
    int rbvIndex;
};

extern "C" {
    int motorSimConfigure(const char *portName);
}

#endif // MOTOR_SIM_DRIVER_HPP
