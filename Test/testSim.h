//
//  testSim.h
//  Hexplore
//
//  Created by Melanie Jouaiti on 14/07/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifndef testSim_h
#define testSim_h

#include <iostream>
#include "Simulation.h"
#include "Odometry.h"

class TestSim: Simulation
{
public:
    TestSim();
    std::vector<double> step();
    void saveToFile(const std::string& path);
    double handleObstacles(const double r);
    void goBackNow();
    void run();
    
private:
    double m_t, m_dt;
    double m_x, m_y, m_theta, m_r;
    Odometry m_odometry;
    std::vector<std::vector<double>> m_boundaries;
    bool m_goBack;
};

#endif /* testSim_h */
