//
//  main.cpp
//  Hexplore
//
//  Created by Melanie Jouaiti on 24/04/2020.
//  Copyright © 2020 Melanie Jouaiti. All rights reserved.
//

#ifdef __APPLE__
#   define _OSX
#elif defined(__linux__)
#   define _LINUX
#endif

#include "../../Grapher/src/Grapher.h"
#include "../Simulation/walk_tripod.h"
#include "../common.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

void key_callback2(GLFWwindow* window, int key, int scancode, int action, int mode);
std::vector<std::string> rhex_names = {"rhex_joint1", "rhex_joint2", "rhex_joint3", "rhex_joint4", "rhex_joint5", "rhex_joint6"};

#ifdef _LINUX
std::string path = "../../Grapher/src/";
#else
std::string path = "/Users/Melanie/Documents/Studies/LORIA/Code/Grapher/src/";
#endif

GLint WIDTH = 1024, HEIGHT = 768;

std::vector<double> coords;
Coord c(0,0,0);


TripodWalk simulation(rhex_names);

int main()
{
    Grapher* grapher;
    grapher = new Grapher(WIDTH, HEIGHT, TMAX, 0.01, 4);
    
    glfwSetKeyCallback(grapher->_Window, key_callback2);
    grapher->setBoundariesX(-23., 23.);
    grapher->setBoundariesY(-23., 23.);
    
    grapher->setDisplayedVariables(0, { std::pair<unsigned int, unsigned int>(0, 1),
                                        std::pair<unsigned int, unsigned int>(2, 3),
                                        std::pair<unsigned int, unsigned int>(4, 5)});
    Shader shader = Shader((path + "shaders/shader.vs").c_str(), (path + "shaders/shader.frag").c_str());

    float t = 0.0, dt = 0.01;

    while (t < TMAX && !grapher->shouldClose())
    {
        std::vector<double> tmp = simulation.step();
        
        coords ={c.x, c.y, tmp[0], tmp[1], tmp[2], tmp[3]};
        grapher->step(coords, shader);
        t += dt;
    }
    
    return 0;
}

void key_callback2(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
         glfwSetWindowShouldClose(window, GL_TRUE);
    
    if (key == GLFW_KEY_UP && action == GLFW_PRESS)
    {
        c += Coord(0,1,0);
        simulation.goUp(3., 1.0, mode == GLFW_MOD_SHIFT);
    }
    else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
    {
        c += Coord(0,-1,0);
        simulation.goDown(3., 1.0, mode == GLFW_MOD_SHIFT);
    }
    else if (key == GLFW_KEY_LEFT && action == GLFW_PRESS)
    {
        c += Coord(-1,0,0);
        simulation.goLeft(3., 1., mode == GLFW_MOD_SHIFT);
    }
    else if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS)
    {
        c += Coord(1,0,0);
        simulation.goRight(3., 1., mode == GLFW_MOD_SHIFT);
    }
    else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        c += Coord(0,0,0);
        simulation.setInitPosition();
        
    }
}
