/** @file test.cpp
 * @brief Test file that consists of all TEST CASES to test the implemented PID Class.
 */
#include <gtest/gtest.h>
#include <PIDController.h>
#include <limits>
#include <gmock/gmock.h>

namespace{
class checkPIDClass : public ::testing::Test {
  protected:
    pidController* pidCon;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double dt = 0.1;
  public:  
    checkPIDClass(){}
    virtual ~checkPIDClass(){}
  
    virtual void SetUp(){
      pidCon = new pidController(kp, ki, kd, dt);
    }
    virtual void TearDown(){
      delete pidCon;
    }
};


TEST_F(checkPIDClass, proportionalCheckTest1){
  double kp = 2.5;
  double dt = 0.1;
  double setPoint = 5.0;
  double feedback = 5.0;
  pidCon->setKpGain(kp);
  pidCon->setDtVal(dt);
  ASSERT_DOUBLE_EQ(dt, pidCon->getDtVal());
  ASSERT_DOUBLE_EQ(kp, pidCon->getKpGain());
  ASSERT_DOUBLE_EQ(0.0, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(12.5, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 0;
  feedback = 5;
  ASSERT_DOUBLE_EQ(-12.5, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = numeric_limits<double>::max();
  feedback = numeric_limits<double>::max();
  ASSERT_DOUBLE_EQ(0, pidCon->calculateVelocity(setPoint, feedback));  
}


TEST_F(checkPIDClass, integralCheckTest){
  double ki = 0.01;
  double dt = 0.1;
  pidCon->setKiGain(ki);
  pidCon->setDtVal(dt);
  ASSERT_DOUBLE_EQ(dt, pidCon->getDtVal());
  ASSERT_DOUBLE_EQ(ki, pidCon->getKiGain());
  double setPoint = 5;
  double feedback = 0;
  ASSERT_DOUBLE_EQ(0.005, pidCon->calculateVelocity(setPoint, feedback));
  ASSERT_DOUBLE_EQ(0.5, pidCon->getIntegralError());
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(0.01, pidCon->calculateVelocity(setPoint, feedback));
  ASSERT_DOUBLE_EQ(1, pidCon->getIntegralError());
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(0.015, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(0.02, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 5;
  ASSERT_DOUBLE_EQ(0.02, pidCon->calculateVelocity(setPoint, feedback));
  ASSERT_DOUBLE_EQ(2, pidCon->getIntegralError());
  pidCon->resetIntegralError();
  ASSERT_DOUBLE_EQ(0, pidCon->getIntegralError());
}


TEST_F(checkPIDClass, derivativeCheckTest){ 
  double kd = 0.1; 
  double dt = 0.1;
  pidCon->setKdGain(kd);
  pidCon->setDtVal(dt);
  ASSERT_DOUBLE_EQ(dt, pidCon->getDtVal());
  ASSERT_DOUBLE_EQ(kd, pidCon->getKiGain());
  double setPoint = 5;
  double feedback = 0;
  ASSERT_DOUBLE_EQ(5.0, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 5;
  ASSERT_DOUBLE_EQ(-5.0, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 5;
  ASSERT_DOUBLE_EQ(0.0, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 0;
  feedback = 5;
  ASSERT_DOUBLE_EQ(-5, pidCon->calculateVelocity(setPoint, feedback));
}

TEST_F(checkPIDClass, combinedTest){
  double kp = -0.1;
  double ki = -0.01;
  double kd = -0.1;
  double dt = 1;
  pidCon->setKpGain(kp);
  pidCon->setKdGain(kd);
  pidCon->setKiGain(ki);
  pidCon->setDtVal(dt);
  ASSERT_DOUBLE_EQ(kd, pidCon->getKdGain());
  ASSERT_DOUBLE_EQ(ki, pidCon->getKiGain());
  ASSERT_DOUBLE_EQ(kp, pidCon->getKpGain());
  ASSERT_DOUBLE_EQ(dt, pidCon->getDtVal());
  double setPoint = -5;
  double feedback = 0;
  ASSERT_DOUBLE_EQ(5.0+0.005+5, pidCon->calculateVelocity(setPoint, feedback));
  setPoint = 5;
  feedback = 0;
  ASSERT_DOUBLE_EQ(-5.0 + 0.0  -10, pidCon->calculateVelocity(setPoint, feedback));
}

TEST_F(checkPIDClass, validDtTest){
  double dt1 = 0.2;
  pidCon->setDtVal(dt1);
  ASSERT_DOUBLE_EQ(dt1, pidCon->getDtVal());
  double dt2 = 0.0;
  pidCon->setDtVal(dt2);
  ASSERT_DOUBLE_EQ(dt1, pidCon->getDtVal());
  double dt3 = -0.1;
  pidCon->setDtVal(dt3);
  ASSERT_DOUBLE_EQ(dt1, pidCon->getDtVal());
}

}