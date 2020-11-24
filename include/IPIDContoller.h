/**
* @file IPIDContoller.h
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
*  Header file which includes the interface or abstract IPIDContoller class
*/

#ifndef INCLUDE_IPIDCONTROLLER_H_
#define INCLUDE_IPIDCONTROLLER_H_

/**
 * @brief IPIDController abstract class
 */
class IPIDController {
 public:
    /**
     * @brief Virtual function for calculating velocity
     * @param double requiredVelocity
     * @param double actualVelocity
     * @return output velocity
     */ 
    virtual calculateVelocity(double requiredVelocity,
                              double actualVelocity) = 0; 
};

#endif