//
//  OrientationController.hpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 07/05/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifndef OrientationController_hpp
#define OrientationController_hpp

#include <iostream>
#include <vector>
#include "common.h"

class OrientationController
{
public:
    
    OrientationController();
    OrientationController(const double Ko, const double Kv, const double maxSpeed);
    std::vector<double> step(const double currentOrientation, const double targetOrientation, const double targetDistance);
    std::vector<double> step(const double currentOrientation, const double targetOrientation, const Coord currentPosition, const Coord targetPosition);
  
    
private:
    double m_Ko, m_Kv;
    double m_maxSpeed;
    
};

#endif /* OrientationController_hpp */
