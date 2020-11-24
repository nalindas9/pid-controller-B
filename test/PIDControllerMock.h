/**
* @file PIDContollerMock.h
* @authors Nalin Das 
*
* @version 1.0
*
* @section LICENSE
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* @section DESCRIPTION
*
*  Mock PIDController class for testing
*/

#ifndef TEST_PIDCONTROLLERMOCK_H_
#define TEST_PIDCONTROLLERMOCK_H_

#include <gmock/gmock.h>
#include "../include/IPIDContoller.h"

class PIDControllerMock : public IPIDController {
 public:
    MOCK_METHOD1(calculateVelocity, double(double requiredVelocity,
                                           double actualVelocity));
};

#endif