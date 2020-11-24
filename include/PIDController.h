/**
* @file PIDContoller.h
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
*  This is header file for PIDController.h this file implements
*  the code to compute the velocity with minimum error and works
*  on feedback loop to maintain velocity to required velocity
*/

#pragma once

#include <iostream>
#include "IPIDController.h"

using namespace std;

/**
 * @brief it is the declaration of pidController Class
 */

class pidController : public virtual IPIDController {
    private:
        double kp; //Proportional gain
        double ki; //Integral gain
        double kd; //Derivative gain
        double dt; //change in time
        double intgrError; // integral of all error over time.  
        double prevError; 
    public:
    
/**
 * @brief default constructor of the class pidController
 * @param dtValue variable for initializing the member dt
 * @return none
 */
    
        pidController();  // Empty Constructor

/**
 * @brief parameterized constructorof class pidController to initialize the private members
 * @param kpValue variable for initializing the member kp
 * @param kiValue variable for initializing the member ki
 * @param kdValue variable for initializing the member kd
 * @param dtValue variable for initializing the member dt
 * @return none
 */ 
    
        pidController(double kpValue, double kiValue, double kdValue, double dtValue); // Value Constructor
    
 /**
 * @brief default destructor which destroys the object of the class pidController
 * @param dtValue variable for initializing the member dt
 * @return none
 */
    
        ~pidController(); // Destructor
 
 /**
 * @brief it is a setter method to set the Kp variable to a new value
 * @param double k
 * @return none
 */   
        void setKpGain(double k); // Update Kp Gain on the fly
  
 /**
 * @brief it is a setter method to set the Ki variable to a new value
 * @param double k
 * @return none
 */  
    
        void setKiGain(double k); // Update Ki Gain on the fly
    
 /**
 * @brief it is a setter method to set the Kd variable to a new value
 * @param double k
 * @return none
 */  
    
        void setKdGain(double k); // Update Kd Gain on the fly

 /**
 * @brief it is a setter method to set the Dt variable to a new value
 * @param double dt
 * @return none
 */    
    
        void setDtVal(double newDt); // Update Dt Val on the fly
  
/**
 * @brief it is a getter method to get the Kp member variable
 * @param none
 * @return 0.0
 */
    
        double getKpGain(); // Get Kp Gain

 /**
 * @brief it is a getter method to get the Ki member variable
 * @param none
 * @return 0.0
 */    
    
        double getKiGain(); // Get Ki Gain

 /**
 * @brief it is a getter method to set the Kd member variable
 * @param none
 * @return 0.0
 */    
    
        double getKdGain(); // Get Kd Gain
  
  /**
 * @brief it is a getter method to get the IntergralError variable
 * @param none
 * @return 0.0
 */  
    
        double getDtVal(); // Get Dt Val
    
/**
 * @brief it is a getter method to get the IntergralError variable
 * @param none
 * @return 0.0
 */
    
        double getIntegralError(); // Get Integral Error value being tracked. 
 
/**
 * @brief it is a method to compute the output velocity
 * @param double requiredVelocity
 * @param double actualVelocity
 * @return output velocity
 */ 
    
        double calculateVelocity(double requiredVelocity, double actualVelocity); // Compute the PID Output
    
/**
 * @brief it is a reset method to reset the Integral Errror
 * @param none
 * @return none
 */    
        void resetIntegralError(); // Reset the Intergrator error
    
    
};
