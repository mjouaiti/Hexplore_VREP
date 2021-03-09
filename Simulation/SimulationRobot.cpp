//
//  SimulationRobot.cpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 14/07/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#include "SimulationRobot.h"

#include "../common.h"
#include "../Robot/Robot_VREP.h"

SimulationRobot::SimulationRobot(): m_robot(), m_t(0),
m_dt(0.01)
{
    
}

SimulationRobot::SimulationRobot(std::vector<std::string> jointNames): m_robot(), m_t(0),
m_dt(0.01)
{
    
    simxFinish(-1);
    m_clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    if (m_clientID!=-1)
    {
        std::cout << "Connected to remote API server" << std::endl;
        simxSetFloatingParameter(m_clientID, sim_floatparam_simulation_time_step, m_dt, simx_opmode_oneshot);
        simxStartSimulation(m_clientID, simx_opmode_blocking);
    }
    else
    {
        std::cerr << "Cannot Connect to remote API server";
        exit(0);
    }
    
    m_robot = new Robot_VREP(m_clientID, jointNames);
    for(unsigned int i = 0; i < jointNames.size(); i++)
        m_PIDControllers.push_back(PIDController(PID_P, PID_I, PID_D));
}

SimulationRobot::~SimulationRobot()
{
    simxStopSimulation(m_clientID, simx_opmode_blocking);
    int pt;
    simxGetPingTime(m_clientID, &pt);
    simxFinish(m_clientID);
    std::cout << "Connection closed"  << std::endl;
}

void SimulationRobot::run()
{
    while (m_t < TMAX)
    {
        step();
    }
}
