//
//  Odometry.cpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 14/07/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#include "Odometry.h"
#include <cmath>


double *velocity, *beta;

Odometry::Odometry():m_wayBackVector(), m_functions()
{
        velocity = new double[1];
        beta = new double[1];
        m_wayBackVector = {0.0000001, PI};
        
    //    std::vector<std::function<double (double)> > inputs {
    //        [this](double t) -> double {return velocity[0];},
    //        [this](double t) -> double {return beta[0];}};
        
        m_functions = std::vector<std::function<double (double, std::vector<double>)> >(0, NULL);
        
        m_functions.push_back([this](double t, std::vector<double> args) -> double {return DR(velocity[0], beta[0]);});
        m_functions.push_back([this](double t, std::vector<double> args) -> double {return DALPHA(velocity[0], args[0], beta[0]);});
        
        m_runge = Integrator((int)m_functions.size(), m_functions, m_wayBackVector);
}

std::vector<double> Odometry::integrate(const double t, const double dt, const double v, const double theta)
{
    velocity[0] = v;
    beta[0] = m_wayBackVector[1] - theta;
    
    m_wayBackVector = m_runge.rk4(t, dt);
    return m_wayBackVector;
}

std::vector<double> Odometry::getVector() const
{
    return m_wayBackVector;
}
