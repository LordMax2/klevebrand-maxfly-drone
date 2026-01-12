#ifndef KLEVEBRAND_MAXFLY_DRONE_H
#define KLEVEBRAND_MAXFLY_DRONE_H

#include <Arduino.h>
#include "base_gyro_drone.h"
#include "servo_drone_motor.h"
#include "./components/quadcopter_pid/quadcopter_pid.h"

class KlevebrandMaxFlyDrone : public BaseGyroDrone<QuadcopterPid>
{
public:
  KlevebrandMaxFlyDrone(ServoDroneMotor* motors) : BaseGyroDrone<QuadcopterPid>() 
  {
    this->motors = motors;
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
  ServoDroneMotor* motors;
  ServoDroneMotor motorLeftFront() { return motors[0]; };
  ServoDroneMotor motorRightFront() { return motors[1]; };
  ServoDroneMotor motorLeftBack() { return motors[2]; };
  ServoDroneMotor motorRightBack() { return motors[3]; };
  void printThrottle();
};

#endif
