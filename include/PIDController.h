/** @file PIDController.h
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

class pidController
{
    private:
        double kp; //Proportional gain
        double ki; //Integral gain
        double kd; //Derivative gain
        double dt; //change in time
        double intgrError; // integral of all error over time.  

    public:
        pidController();  // Empty Constructor  
        pidController(double kpValue, double kiValue, double kdValue, double dtValue); // Value Constructor
        ~pidController(); // Destructor
        void setKpGain(double k); // Update Kp Gain on the fly
        void setKiGain(double k); // Update Ki Gain on the fly
        void setKdGain(double k); // Update Kd Gain on the fly
        void setDtVal(double newDt); // Update Dt Val on the fly
        double getKpGain(); // Get Kp Gain
        double getKiGain(); // Get Ki Gain
        double getKdGain(); // Get Kd Gain
        double getDtVal(); // Get Dt Val
        double getIntegralError(); // Get Integral Error value being tracked. 
        double calculateVelocity(double requiredVelocity, double actualVelocity); // Compute the PID Output
        void resetIntegralError(); // Reset the Intergrator error
};
