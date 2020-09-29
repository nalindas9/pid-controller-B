/** @file test.cpp
 * @brief Test file that consists of all TEST CASES to test the implemented PID Class.
 * @brief Makes use of Google mock's Setup and TearDown features for testing classes.
 */
#include <gtest/gtest.h>
#include <PIDController.h>
#include <limits>
#include <gmock/gmock.h>

namespace
{
  /**
   * @brief Class checkPIDClass is instantiated before every TEST_F call which uses checkPIDClass as a parameter. 
   * @brief This helps testing classes easy and reduces lines of code required to instantiate variables each time. 
  */
  class checkPIDClass : public ::testing::Test
  {
  protected: // default variables
    pidController *pidCon;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double dt = 0.1;

  public:
    checkPIDClass() {}          // constructor
    virtual ~checkPIDClass() {} // destructor

    virtual void SetUp() // required method that is called at the start of every TEST_F
    {
      pidCon = new pidController(kp, ki, kd, dt); // Use of non empty constructor
    }
    virtual void TearDown() // required method that is called at the end of every TEST_F
    {
      delete pidCon;
    }
  };

  /**
   * @brief This block tests the proportional part of PID control block. 
   * @brief Except Kp gain, all gains are set to be 0. 
   * @brief The methods setKpGain, getKpGain, setDtVal, getDtVal, calculateVelocity are evalulated here.
   * @param None
   * @return None
  */
  TEST_F(checkPIDClass, proportionalCheckTest1)
  {
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

  /**
   * @brief This block tests the Integral part of PID control block. 
   * @brief Except Ki gain, all gains are set to be 0. 
   * @brief The methods setKiGain, getKiGain, setDtVal, getDtVal, calculateVelocity are evalulated here.
   * @param None
   * @return None
  */
  TEST_F(checkPIDClass, integralCheckTest)
  {
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

  /**
   * @brief This block tests the Derivative part of PID control block. 
   * @brief Except Kd gain, all gains are set to be 0. 
   * @brief The methods setKdGain, getKdGain, setDtVal, getDtVal, calculateVelocity are evalulated here.
   * @param None
   * @return None
  */
  TEST_F(checkPIDClass, derivativeCheckTest)
  {
    double kd = 0.1;
    double dt = 0.1;
    pidCon->setKdGain(kd);
    pidCon->setDtVal(dt);
    ASSERT_DOUBLE_EQ(dt, pidCon->getDtVal());
    ASSERT_DOUBLE_EQ(kd, pidCon->getKdGain());
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

  /**
   * @brief This block tests the combined PID control block. 
   * @brief All three values are set to be non-zero and evaluated. 
   * @brief The methods set and get are evalulated here.
   * @param None
   * @return None
  */
  TEST_F(checkPIDClass, combinedTest)
  {
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
    ASSERT_DOUBLE_EQ(.50 + 0.05 + .5, pidCon->calculateVelocity(setPoint, feedback));
    setPoint = 5;
    feedback = 0;
    ASSERT_DOUBLE_EQ(-.50 + 0.0 - 1, pidCon->calculateVelocity(setPoint, feedback));
  }

  /**
   * @brief This block tests the valid values of dt.
   * @brief Specifically tests the function getDtVal and setDtVal 
   * @brief Except Ki gain, all gains are set to be 0. 
   * @brief The methods setKiGain, getKiGain, setDtVal, getDtVal, calculateVelocity are evalulated here.
   * @param None
   * @return None
  */
  TEST_F(checkPIDClass, validDtTest)
  {
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

  TEST(pidControllerTest, classInitialisationTest){
    pidController pidCon(0.1, 0.1, 0.1, 0.1);
    ASSERT_DOUBLE_EQ(0.1, pidCon.getKdGain());
    ASSERT_DOUBLE_EQ(0.1, pidCon.getKiGain());
    ASSERT_DOUBLE_EQ(0.1, pidCon.getKpGain());
    ASSERT_DOUBLE_EQ(0.1, pidCon.getDtVal());
  }

} // namespace