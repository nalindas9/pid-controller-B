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

#include<iostream>
#include "PIDController.h"

/**
 * @brief it is the main function to implement PID Controller
 * @param none
 * @returns 0
 */

int main() {
    pidController pid(0.02, 0.002, 0.002, 0.2);     // Initializing the constructor with the input values of kp, ki, kd, dt
    auto controlOutput = pid.calculateVelocity(5.0,10.0);       // Calling the calculate velocity method
    cout<<"The Control output is : "<<  controlOutput << " units"<<endl;
}
