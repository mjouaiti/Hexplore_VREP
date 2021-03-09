//
//  walk_tripod.hpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 24/04/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifndef walk_tripod_hpp
#define walk_tripod_hpp

#include <stdio.h>
#include <iostream>
#include <queue>

#include "../common.h"
#include "SimulationRobot.h"
#include "OrientationController.h"

class TripodWalk: SimulationRobot
{
public:
    TripodWalk();
    TripodWalk(std::vector<std::string> jointNames);
    ~TripodWalk();
    virtual std::vector<double> step();
    virtual void saveToFile(const std::string& path);
    void goRight(const double speed, const double targetDistance, const bool backwards);
    void goLeft(const double speed, const double targetDistance, const bool backwards);
    void goUp(const double speed, const double targetDistance, const bool backwards);
    void goDown(const double speed, const double targetDistance, const bool backwards);
    void setInitPosition();
    
private:
    std::vector<std::string> m_jointNames;
    std::vector<double> m_velocities;
    std::queue<Goal> m_goals;
    double m_targetDistance;
    Coord m_goalEndpoint;
    
    OrientationController m_controller;
    std::vector<double> m_record;               /**< recorded values */
};

#endif /* walk_tripod_hpp */
