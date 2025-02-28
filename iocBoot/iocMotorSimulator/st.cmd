#!/app/MotorSimulatorApp/bin/linux-x86_64/MotorSimulator

#- SPDX-FileCopyrightText: 2003 Argonne National Laboratory
#-
#- SPDX-License-Identifier: EPICS

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "${TOP}/dbd/MotorSimulator.dbd"
MotorSimulator_registerRecordDeviceDriver pdbbase

## Configure the motor simulator driver
motorSimConfigure("MOTOR_PORT")

## Load record instances for two devices
dbLoadRecords("${TOP}/Db/motor1.db", "PORT=MOTOR_PORT")
dbLoadRecords("${TOP}/Db/motor2.db", "PORT=MOTOR_PORT")

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncxxx,"user=libojiang"
