#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "template_gyro_drone.h"
#include "servo_drone_motor.h"
#include "quadcopter_pid.h"
#include "bno08x_drone_gyro.h"
#include "eeprom_pid_repository.h"

class KlevebrandMaxFlyDrone : public TemplateGyroDrone<QuadcopterPid, Bno08xDroneGyro, EepromPidRepository>
{
private:
  ServoDroneMotor *_motors;
  ServoDroneMotor& motorLeftFront() { return _motors[0]; };
  ServoDroneMotor& motorRightFront() { return _motors[1]; };
  ServoDroneMotor& motorLeftBack() { return _motors[2]; };
  ServoDroneMotor& motorRightBack() { return _motors[3]; };
  Bno08xDroneGyro _gyro;
  EepromPidRepository _pid_repository;
  void printThrottle();

public:
  KlevebrandMaxFlyDrone(ServoDroneMotor *motors) : TemplateGyroDrone<QuadcopterPid, Bno08xDroneGyro, EepromPidRepository>(500, 200, 10000, &_gyro), _gyro(10)
  {
    this->_motors = motors;
  }

  void setup() override;
  void run() override;
  void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;
  void setupMotors() override
  {
    motorLeftFront().setSpeed(0);
    motorRightFront().setSpeed(0);
    motorLeftBack().setSpeed(0);
    motorRightBack().setSpeed(0);

    delay(1000);
  };
  void stopMotors() override
  {
    motorLeftFront().setSpeed(0);
    motorRightFront().setSpeed(0);
    motorLeftBack().setSpeed(0);
    motorRightBack().setSpeed(0);
  };
};

#endif
