/**
* @file PIDController.cpp
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
*  This is cpp file for PIDController.cpp this file implements
*  the code o compute the velocity with minimum error and works
*  on feedback loop to maintain velocity to required velocity
*/

#include "../include/PIDController.h"

pidController::pidController(double kpValue, double kiValue, double kdValue, double dtValue) {
    kp = kpValue;
    ki = kiValue;
    kd = kdValue;
    dt = dtValue;
}
double pidController::calculateVelocity(double requiredVelocity, double actualVelocity) {
    double currentError = requiredVelocity - actualVelocity;
    double previousError = 0.0, integral = 0.0, derivative = 0.0, finalOutput = 0.0;
    int count = 0;
    derivative = (currentError - previousError)/ dt;
    while (count < 150000){
        integral = integral + (currentError * dt);
        finalOutput = kp * currentError + ki * integral + kd * derivative;
        previousError = currentError;
        if (requiredVelocity - 0.1 < finalOutput && finalOutput < requiredVelocity + 0.1)
            break;
        actualVelocity = finalOutput;
        count ++;
        currentError = requiredVelocity - actualVelocity;
        derivative = (currentError - previousError)/ dt;
    }
    if (count == 150000)
        cout << "Limit reached" << endl;
    return finalOutput;
}

double pidController::changeInTime(double newDt) {
    dt = newDt;
    return dt;
}


pidController::~pidController() {}

