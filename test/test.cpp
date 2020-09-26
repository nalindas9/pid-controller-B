#include <gtest/gtest.h>
#include <PIDController.h>
#include <main.cpp>
#include <limits>

TEST(checkCalcVelocity, proportionalCheckTest){
  double kp = 2.5; 
  double ki = 0.00; 
  double kd = 0; 
  double dt = 0.1; 
  pidController pidCon = pidController(kp, ki, kd, dt);
  auto setPoint = 5;
  auto feedback = 5;
  ASSERT_DOUBLE_EQ(0.0, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(12.5, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = 0;
  feedback = 5;
  ASSERT_DOUBLE_EQ(-12.5, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = numeric_limits<double>::max();
  feedback = numeric_limits<double>::max();
  ASSERT_DOUBLE_EQ(0.0, pidCon.calculateVelocity(setPoint, feedback));
}


TEST(checkCalcVelocity, integralCheckTest){
  double kp = 0.0; 
  double ki = 0.01; 
  double kd = 0; 
  double dt = 0.1; 
  pidController pidCon = pidController(kp, ki, kd, dt);
  auto setPoint = 5;
  auto feedback = 0;
  ASSERT_DOUBLE_EQ(0.005, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(0.01, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(0.015, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(0.02, pidCon.calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 5;
  ASSERT_DOUBLE_EQ(0.02, pidCon.calculateVelocity(setPoint, feedback));
}

