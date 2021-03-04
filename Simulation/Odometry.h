//
//  Odometry.hpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 14/07/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifndef Odometry_h
#define Odometry_h

#include <stdio.h>
#include <vector>
#include "../lib-cpg/Integrator.h"

#define DR(v, beta) - v * cos(beta)
#define DALPHA(v, r, beta) v / r * sin(beta)
#define PI acos(-1)

class Odometry
{
public:
    Odometry();
    std::vector<double> integrate(const double t, const double dt, const double velocity, const double theta);
    std::vector<double> getVector() const;
    
private:
    std::vector<double> m_wayBackVector;
    std::vector<std::function<double (double, std::vector<double>)>> m_functions;
    Integrator m_runge;
    
};

#endif /* Odometry_h */
