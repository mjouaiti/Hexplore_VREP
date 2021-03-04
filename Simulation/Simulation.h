//
//  Simulation.hpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 24/04/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifndef Simulation_hpp
#define Simulation_hpp

#include <stdio.h>
#include <vector>
#include <map>

#define PI acos(-1)

class Simulation
{
public:
    Simulation();
    Simulation(std::vector<std::string> jointNames);
    ~Simulation();
    
    virtual std::vector<double> step() = 0;
    virtual void run() = 0;
    virtual void saveToFile(const std::string& path) = 0;

protected:
    
};

#endif /* Simulation_hpp */
