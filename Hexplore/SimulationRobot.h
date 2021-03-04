//
//  SimulationRobot.hpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 14/07/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifndef SimulationRobot_hpp
#define SimulationRobot_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include "Simulation.h"
#include "PIDController.h"
#include "CPG.h"
#include "../Robot/Robot.h"
#include "Odometry.h"

#endif /* SimulationRobot_hpp */

class SimulationRobot: Simulation
{
public:
    SimulationRobot();
    SimulationRobot(std::vector<std::string> jointNames);
    ~SimulationRobot();
    
    virtual std::vector<double> step() = 0;
    void run();
    virtual void saveToFile(const std::string& path) = 0;

protected:
    double m_t, m_dt;
    std::vector<PIDController> m_PIDControllers;
    std::map<std::string, CPG*> m_CPGs;
    Robot* m_robot;
    int m_clientID;                             /**< id of the VREP client */
    Odometry m_odometry;
    
};


