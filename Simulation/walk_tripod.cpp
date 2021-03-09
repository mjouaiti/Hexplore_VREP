//
//  walk_tripod.cpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 24/04/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#include "walk_tripod.h"
#include "../Robot/Robot_VREP.h"
#include <fstream>

#define UP 0

TripodWalk::TripodWalk(): m_goalEndpoint(0, 0, 0)
{
    
}

TripodWalk::TripodWalk(std::vector<std::string> jointNames):SimulationRobot(jointNames), m_jointNames(jointNames), m_targetDistance(0.0), m_goalEndpoint(0, 0, 0)
{
    m_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_controller = OrientationController(0.05, 0.5, 3.0);
}

TripodWalk::~TripodWalk()
{
    saveToFile("");
}

void TripodWalk::setInitPosition()
{
//    m_robot->setPositionControl();
//    for(unsigned int i=UP; i < 6; i+=2)
//        m_robot->setPosition(i, 180.0);
//    for(unsigned int i=!UP; i < 6; i+=2)
//        m_robot->setPosition(i, 0.0);
//
//    m_robot->setVelocityControl();
}

std::vector<double> TripodWalk::step()
{
    std::vector<double> temp;
    
    Coord c = dynamic_cast<Robot_VREP*>(m_robot)->getCOMPosition();
    Coord a = dynamic_cast<Robot_VREP*>(m_robot)->getCOMOrientation();
    double v = dynamic_cast<Robot_VREP*>(m_robot)->getCOMVelocity();
    
    m_record.push_back(m_t);
    m_record.push_back(c.x);
    m_record.push_back(c.y);
    m_record.push_back(c.z);
    m_record.push_back(a.x);
    m_record.push_back(a.y);
    m_record.push_back(a.z);
    
    
    if(!m_goals.empty())
        m_targetDistance = m_goals.front()._targetDistance;
    else
        m_targetDistance = 0.0;
    
    std::cout << m_targetDistance << std::endl;
    
    if(m_targetDistance <= 0.5 && !m_goals.empty())
    {
        m_velocities = {.0, .0, .0, .0, .0, .0};
        setInitPosition();
        m_goals.pop();
    }

    if(!m_goals.empty())
    {
        m_velocities = m_controller.step(a.z, m_goals.front()._targetOrientation, m_targetDistance);
        std::cout << c << " << " << m_goals.front()._goalPoint << std::endl;
        m_goals.front()._targetDistance -= v * m_dt;
    }
    
    int i = 0;
    for(auto name:m_jointNames)
    {
        m_robot->setVelocity(name, m_velocities[i]);
        i++;
    }
    
    
    std::vector<double> var = m_odometry.integrate(m_t, m_dt, v, a.z / 180.0 * PI);
    m_t += m_dt;
    std::cout << m_t << "//" << v << std::endl;
    
    temp.push_back(c.x);
    temp.push_back(c.y);
    temp.push_back(c.x + var[0] * cos(var[1]));
    temp.push_back(c.y + var[0] * sin(var[1]));
    return temp;
}

void TripodWalk::goLeft(const double speed, const double targetDistance, const bool backwards)
{
    if(!m_goals.empty())
    {
            Coord goal = m_goals.front()._goalPoint + Coord(1, 0, 0) * targetDistance;
            if(m_goals.front()._targetOrientation == 90.0)
            {
                m_goals.front()._targetDistance += targetDistance;
                m_goals.front()._goalPoint = goal;
            }
            else
                m_goals.push(Goal(targetDistance, 90.0, goal));
        }
    else
    {
            Coord c = dynamic_cast<Robot_VREP*>(m_robot)->getCOMPosition();
            Coord goal = c + Coord(1, 0, 0) * targetDistance;
            m_goals.push(Goal(targetDistance, 90.0, goal));
    }
    if(backwards)
        m_velocities = {-speed, -speed, -speed, -speed/2., -speed/2., -speed/2.};
    else
        m_velocities = {speed, speed, speed, speed/2., speed/2., speed/2.};
}

void TripodWalk::goRight(const double speed, const double targetDistance, const bool backwards)
{
    if(!m_goals.empty())
    {
        Coord goal = m_goals.front()._goalPoint + Coord(-1, 0, 0) * targetDistance;
        if(m_goals.front()._targetOrientation == -90.0)
        {
            m_goals.front()._targetDistance += targetDistance;
            m_goals.front()._goalPoint = goal;
        }
        else
            m_goals.push(Goal(targetDistance, -90.0, goal));
   }
    else
    {
       Coord c = dynamic_cast<Robot_VREP*>(m_robot)->getCOMPosition();
       Coord goal = c + Coord(-1, 0, 0) * targetDistance;
        m_goals.push(Goal(targetDistance, -90.0, goal));
    }
    if(backwards)
        m_velocities = {-speed/2., -speed/2., -speed/2., -speed, -speed, -speed};
    else
        m_velocities = {speed/2., speed/2., speed/2., speed, speed, speed};
}

void TripodWalk::goUp(const double speed, const double targetDistance, const bool backwards)
{
    if(!m_goals.empty())
    {
        Coord goal = m_goals.front()._goalPoint + Coord(0, 1, 0) * targetDistance;
        if(m_goals.front()._targetOrientation == 0.0)
        {
            m_goals.front()._targetDistance += targetDistance;
            m_goals.front()._goalPoint = goal;
        }
        else
            m_goals.push(Goal(targetDistance, 0.0, goal));
    }
    else
    {
        Coord c = dynamic_cast<Robot_VREP*>(m_robot)->getCOMPosition();
        Coord goal = c + Coord(0, 1, 0) * targetDistance;
        m_goals.push(Goal(targetDistance, 0.0, goal));
    }
    if(backwards)
        m_velocities = {-speed, -speed, -speed, -speed, -speed, -speed};
    else
        m_velocities = {speed, speed, speed, speed, speed, speed};
    
    std::cout << "Gooal point: " << m_goals.front()._goalPoint << std::endl;
}

void TripodWalk::goDown(const double speed, const double targetDistance, const bool backwards)
{
    if(!m_goals.empty())
    {
        Coord goal = m_goals.front()._goalPoint + Coord(0, -1, 0) * targetDistance;
        if(m_goals.front()._targetOrientation == 180.0)
        {
            m_goals.front()._targetDistance += targetDistance;
            m_goals.front()._goalPoint = goal;
        }
        else
            m_goals.push(Goal(targetDistance, 180.0, goal));
    }
    else
    {
        Coord c = dynamic_cast<Robot_VREP*>(m_robot)->getCOMPosition();
        Coord goal = c + Coord(0, -1, 0) * targetDistance;
        m_goals.push(Goal(targetDistance, 180.0, goal));
    }
    if(backwards)
        m_velocities = {-speed, -speed, -speed, -speed, -speed, -speed};
    else
        m_velocities = {speed, speed, speed, speed, speed, speed};
}


void TripodWalk::saveToFile(const std::string& path)
{
    std::string fileName = "test.csv";
    std::ofstream out(path + fileName, std::ios::out);
    if(!out.is_open())
        std::cerr << "Could not open file: " << path + fileName << std::endl;
    else
    {
        std::string header = "t,COM_x,COM_y,COM_z,or_x,or_y,or_z";
        int nb = (int)std::count(header.begin(), header.end(), ',') + 1;
        out << header << std::endl;
        for(unsigned int i = 0; i < m_record.size(); i++)
            out << std::to_string(m_record[i]) + (((i+1) % nb == 0)? "\n": ",");
    }
    out.close();
}
