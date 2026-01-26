#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "base_gyro_drone.h"
#include "servo_drone_motor.h"
#include "quadcopter_pid.h"

class KlevebrandMaxFlyDrone : public BaseGyroDrone<QuadcopterPid>
{
public:
  KlevebrandMaxFlyDrone(ServoDroneMotor* motors) : BaseGyroDrone<QuadcopterPid>(500, 200, 10000)
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
  };
  void stopMotors() override
  {
    motorLeftFront().setSpeed(0);
    motorRightFront().setSpeed(0);
    motorLeftBack().setSpeed(0);
    motorRightBack().setSpeed(0);
  };

private:
  ServoDroneMotor* _motors;
  ServoDroneMotor motorLeftFront() { return _motors[0]; };
  ServoDroneMotor motorRightFront() { return _motors[1]; };
  ServoDroneMotor motorLeftBack() { return _motors[2]; };
  ServoDroneMotor motorRightBack() { return _motors[3]; };
  void printThrottle();
};

#endif
