/**
* @file PIDController.cpp
* @author
*
* Karan Sutradhar (117037272)
* Vishnuu Appaya Dhanabalan (116873314)
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
*  This is cpp file for PIDController.cpp this file implements
*  the code to compute the velocity with minimum error and works
*  on feedback loop to maintain velocity to required velocity
*/

#include "PIDController.h"

/**
 * @brief parameterized constructor to initialize the private members
 * @param textInput is the user input text
 * @param stringToSearch is the string to search in the given text
 * @returns location of string in the text in size_t
 */

pidController::pidController(){}

pidController::pidController(double kpValue, double kiValue, double kdValue, double dtValue) {
    kp = kpValue;
    ki = kiValue;
    kd = kdValue;
    dt = dtValue;
}

pidController::~pidController(){}

void pidController::setKpGain(double k){}

void pidController::setKiGain(double k){}

void pidController::setKdGain(double k){}

void pidController::setDtVal(double dt){}

double pidController::getDtVal(){return 0.0;}

double pidController::getKpGain(){return 0.0;}

double pidController::getKiGain(){return 0.0;}

double pidController::getKdGain(){return 0.0;}

double pidController::getIntegralError(){return 0.0;}

void pidController::resetIntegralError(){}

double pidController::calculateVelocity(double requiredVelocity, double actualVelocity) {
    return 0.0;
}
