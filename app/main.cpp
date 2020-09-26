/**
* @file main.cpp
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
*  This is main.cpp file for printing position this file implements
*  the code to o compute the velocity with minimum error and works
*  on feedback loop to maintain velocity to required velocity
*/

#include<iostream>
#include "../include/PIDController.h"

/**
 * @brief it is the main function to print the position of the string in given text
 * @returns 0
 */

int main() {
    pidController pid(0.02, 0.002, 0.002, 0.2);
    auto controlOutput = pid.calculateVelocity(5.0,10.0);
    cout<<"The Control output is : "<<  controlOutput << " units"<<endl;
}
