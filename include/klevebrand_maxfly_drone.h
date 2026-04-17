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
  static constexpr int motor_pin_count = 4;
  int _motor_pins[motor_pin_count];
  ServoDroneMotor &motorLeftFront() { return _motors[0]; };
  ServoDroneMotor &motorRightFront() { return _motors[1]; };
  ServoDroneMotor &motorLeftBack() { return _motors[2]; };
  ServoDroneMotor &motorRightBack() { return _motors[3]; };
  Bno08xDroneGyro _gyro;
  BasePidRepository _pid_repository;
  BaseDronePosition _position;
  HardwareProcessorArduino _processor;
  void printThrottle();
  void attachMotors();
  void detachMotors();

public:
  KlevebrandMaxFlyDrone(ServoDroneMotor *motors, const int motor_pins[motor_pin_count]) : TemplateDrone<QuadcopterPid>(500, 200, 10000, &_processor, &_gyro, &_pid_repository, &_position), _gyro(10)
  {
    this->_motors = motors;
    for (int i = 0; i < motor_pin_count; i++)
    {
      _motor_pins[i] = motor_pins[i];
    }
  }

  void setup() override;
  bool run() override;
  void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw) override;
  void enableMotors();
  void disableMotors();
  void setupMotors() override
  {
    for (int i = 0; i < motor_pin_count; i++)
    {
      _motors[i].setup(_motor_pins[i]);
    }

    stopMotors();

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
