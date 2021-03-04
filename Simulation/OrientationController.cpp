//
//  OrientationController.cpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 07/05/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#include "OrientationController.h"
#define PI acos(-1)

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

OrientationController::OrientationController(): m_Ko(1.5), m_Kv(1.), m_maxSpeed(4.0)
{
    
}

OrientationController::OrientationController(const double Ko, const double Kv, const double maxSpeed): m_Ko(Ko), m_Kv(Kv), m_maxSpeed(maxSpeed)
{
    
}

std::vector<double> OrientationController::step(const double currentOrientation, const double targetOrientation, const double targetDistance)
{
    double v = m_Kv * targetDistance;
    
//    double bias = atan((targetPosition.y - currentPosition.y) / (targetPosition.x - currentPosition.x)) * 180.0 / PI;
    double theta = m_Ko * (targetOrientation - currentOrientation);//
    
    double even_v = std::min(v, m_maxSpeed);
    double odd_v = std::min(v - theta * v, m_maxSpeed + 1);
    
    std::cout << "theta: " << ";" << currentOrientation << ";" << targetOrientation << std::endl;
    std::cout << "speed: " << odd_v << ", " << even_v << std::endl;

    return {odd_v, odd_v, odd_v, even_v, even_v, even_v};
    
    
}
