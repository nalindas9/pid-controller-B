/**
* @file PIDController.h
* @author
*
* Karan Sutradhar (117037272)
* Vishnuu Appaya Dhanabalan
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
*  This is header file for PIDController.h this file implements
*  the code to compute the velocity with minimum error and works
*  on feedback loop to maintain velocity to required velocity
*/

#pragma once

#include <iostream>
using namespace std;

class pidController{
private:
    double kp;      //Proportional gain
    double ki;      //Integral gain
    double kd;      //Derivative gain
    double dt;        //change in time


public:
    pidController();
    pidController(double kpValue, double kiValue, double kdValue, double dtValue);
    ~pidController();
    double calculateVelocity(double requiredVelocity, double actualVelocity);
    double changeInTime(double newDt);
};
















/**
 * @brief finds the position of a given string in a given text
 * @param textInput is the user input text
 * @param stringToSearch is the string to search in the given text
 * @returns location of string in the text in size_t
 */



