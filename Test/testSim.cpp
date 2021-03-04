//
//  testSim.cpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 14/07/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#include "testSim.h"
#include <cmath>


bool collision(const double x, const double y)
{
    return false;
}

TestSim::TestSim(): m_x(0), m_y(0), m_theta(0.0), m_t(0.0), m_dt(0.01)
{
    m_boundaries = {{-100, 100}, {-100, 100}};
}

std::vector<double> TestSim::step()
{
    std::vector<double> temp;
    
    if(!m_goBack)
    {
        m_theta += (rand()% 20 - 10) / 180.0  * PI;
        m_r = rand() % 2 + 1;
        
        m_r = handleObstacles(m_r);
        
        m_x += m_r * cos(m_theta);
        m_y += m_r * sin(m_theta);
        
        std::vector<double> var = m_odometry.integrate(m_t, m_dt, m_r / m_dt, m_theta);
        
        m_t += m_dt;
        
        temp.push_back(m_x);
        temp.push_back(m_y);
        temp.push_back(m_x + var[0] * cos(var[1]));
        temp.push_back(m_y + var[0] * sin(var[1]));
        temp.push_back(0);
        temp.push_back(0);
    }
    else
    {
        if(m_r > 0.1)
        {
            double dr = m_r * m_dt;
            m_x += dr * cos(m_theta);
            m_y += dr * sin(m_theta);
            m_r -= dr;
            temp.push_back(0);
            temp.push_back(0);
            temp.push_back(0);
            temp.push_back(0);
            temp.push_back(m_x);
            temp.push_back(m_y);
        }
    }
    
    return temp;
}

double TestSim::handleObstacles(const double r)
{
// First: check boundaries
    double tempX = m_x + r * cos(m_theta);
    double tempY = m_y + r * sin(m_theta);
    double tempR = r;
    
    while(tempX < m_boundaries[0][0] || tempX > m_boundaries[0][1] || tempY < m_boundaries[1][0] || tempY > m_boundaries[1][1] || collision(tempX, tempY))
    {
        m_theta += (rand()% 20 - 10) / 180.0  * PI;
        tempR = rand() % 2 + 1;

        tempX = m_x + r * cos(m_theta);
        tempY = m_y + r * sin(m_theta);
    }
    return tempR;
}


void TestSim::saveToFile(const std::string& path)
{
    
}

void TestSim::goBackNow()
{
    m_goBack = true;
    m_theta = m_odometry.getVector()[1];
    m_r = m_odometry.getVector()[0];
}

void TestSim::run()
{
    while (m_t < 10)
    {
        step();
    }
}
