/**
* @file main.cpp
* @authors
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
*  This is main.cpp file for PID Controller this file implements
*  the code to compute the velocity with minimum error and works
*  on feedback loop to maintain velocity to required velocity
*/

#include <iostream>
#include "PIDController.h"

/**
 * @brief It is the main function to demonstrate working of implemented controller.
 * @param none
 * @returns 0
 */

int main()
{
    double kp = 0.02;
    double ki = 0.002;
    double kd = 0.002;
    double dt = 0.2;
    double setPoint = 5;
    double feedback = 0;
    double tolerance = 0.1;
    pidController pid(kp, ki, kd, dt); // Initializing the constructor with the input values of kp, ki, kd, dt
    std::cout << "PID GAINS For the simulation" << std::endl;
    std::cout << "kp: " << kp << ", ki: " << ki << ", kd: " << kd << ", dt: " << dt << std::endl;
    std::cout << "Error Tolerance: " << tolerance << std::endl;
    std::cout << "SetPoint Velocity: " << setPoint << ", Initial Velocity: " << feedback << std::endl;

    while (abs(setPoint - feedback) > abs(tolerance))
    {
        feedback = pid.calculateVelocity(setPoint, feedback);
        std::cout << "Feedback: " << feedback << std::endl;
    }
    std::cout << std::endl
              << "Controller Converged!" << std::endl;
}
