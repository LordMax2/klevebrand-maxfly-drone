#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "template_drone.h"
#include "servo_drone_motor.h"
#include "quadcopter_pid.h"
#include "bno08x_drone_gyro.h"
#include "hardware_processor_arduino.h"

class KlevebrandMaxFlyDrone : public TemplateDrone<QuadcopterPid>
{
private:
  ServoDroneMotor *_motors;
  ServoDroneMotor &motorLeftFront() { return _motors[0]; };
  ServoDroneMotor &motorRightFront() { return _motors[1]; };
  ServoDroneMotor &motorLeftBack() { return _motors[2]; };
  ServoDroneMotor &motorRightBack() { return _motors[3]; };
  Bno08xDroneGyro _gyro;
  BasePidRepository _pid_repository;
  BaseDronePosition _position;
  HardwareProcessorArduino _processor;
  void printThrottle();

public:
  KlevebrandMaxFlyDrone(ServoDroneMotor *motors) : TemplateDrone<QuadcopterPid>(500, 200, 10000, &_processor, &_gyro, &_pid_repository, &_position), _gyro(10)
  {
    this->_motors = motors;
  }

  void setup() override;
  bool run() override;
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
