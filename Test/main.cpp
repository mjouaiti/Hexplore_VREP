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
#include "testSim.h"
#include "../common.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

void key_callback2(GLFWwindow* window, int key, int scancode, int action, int mode);

#ifdef _LINUX
std::string path = "../../Grapher/src/";
#else
std::string path = "/Users/Melanie/Documents/Studies/LORIA/Code/Grapher/src/";
#endif

GLint WIDTH = 1024, HEIGHT = 768;

TestSim simulation;

int main()
{
    Grapher* grapher;
    grapher = new Grapher(WIDTH, HEIGHT, TMAX, 0.01, 4);
    glfwSetKeyCallback(grapher->_Window, key_callback2);
    
    grapher->setBoundariesX(-100., 100.);
    grapher->setBoundariesY(-100., 100.);
    
    grapher->setDisplayedVariables(0, { std::pair<unsigned int, unsigned int>(0, 1),
                                        std::pair<unsigned int, unsigned int>(2, 3),
                                        std::pair<unsigned int, unsigned int>(4, 5)});
    Shader shader = Shader((path + "shaders/shader.vs").c_str(), (path + "shaders/shader.frag").c_str());

    float t = 0.0, dt = 0.01;

    while (t < TMAX && !grapher->shouldClose())
    {
        
        grapher->step(simulation.step(), shader);
        t += dt;
    }
    
    return 0;
}


void key_callback2(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
         glfwSetWindowShouldClose(window, GL_TRUE);
    
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        simulation.goBackNow();
    }
}
