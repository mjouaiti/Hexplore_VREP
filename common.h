//
//  common.h
//  Handshaking
//
//  Created by Melanie Jouaiti on 07/10/2017.
//  Copyright © 2017 Melanie Jouaiti. All rights reserved.
//

#ifndef common_h
#define common_h

#include <cmath>
#include <ostream>

struct Coord
{
    double x;
    double y;
    double z;
    Coord(double x, double y, double z):x(x), y(y), z(z) {}
    Coord(float tmp[3]):x(tmp[0]), y(tmp[1]), z(tmp[2]) {}
    Coord operator+(const Coord c1)
    {
        return Coord(c1.x+x, c1.y+y, c1.z+z);
    }
    
    Coord operator-(const Coord c1)
    {
        return Coord(x-c1.x, y-c1.y, z-c1.z);
    }
    
    Coord operator*(const double c)
    {
        return Coord(x*c, y*c, z*c);
    }
    
    void operator+=(const Coord c1)
    {
        x += c1.x;
        y += c1.y;
        z += c1.z;
    }
    
    double norm()
    {
        return sqrt(x*x+y*y+z*z);
    }
    
    double norm2()
    {
        return sqrt(x*x+y*y);
    }
};

struct Goal
{
    double _targetDistance;
    double _targetOrientation;
    Coord _goalPoint;
    
    Goal(const double targetDist, const double targetOrient, const Coord goal): _targetDistance(targetDist), _targetOrientation(targetOrient), _goalPoint(goal){}
};

std::ostream& operator<<(std::ostream& os, const Coord& c);
double dist(const Coord c1, const Coord c2);

#define PID_P 0.05
#define PID_I 0
#define PID_D 0

#define TMAX 110

#define JOINT_NB1 1
#define JOINT_NB2 2

//------ Rowat-Selverston
#define _VEQ(y, yother, input, eps, W) (y - W*y/(1 + exp(-4*yother)) + /*eps * */input(t))
#define _YEQ(V, y, ss, tauM, tauS, sigmaF, Af) (1 / tauM * (sigmaF - tauM/tauS - 1.0 - sigmaF * pow(tanh(sigmaF * V / Af), 2))*y - (1.0 + ss)/(tauS*tauM)*V + Af * tanh(sigmaF * V / Af) / (tauS*tauM))
#define _SSEQ(V, y, ss, input, tauM, tauS, sigmaF, eps) /*(1 - sigmaF) * */(2 * eps * input(t) * sqrt(tauM * tauS * (1 + ss - sigmaF))*y/r(V, y))
#define _PHIEQ(V0, y0, V, y, phi) sin(theta(V0, y0) - theta(V, y) - phi)
#define _AFEQ(V, y, input, Af, eps, sigmaF, mu) -mu * (400 * pow(V / Af * sigmaF, 2) - pow(input(t), 2))
#define _EPSEQ(input, eps, lambda) lambda * tanh(std::abs(100.0 * input(t))) * (1.0 - pow(input(t)*eps, 2))

//----- Matsuoka
#define M_UEQ(u, v, y_other, input)((- u - M_W * y_other - M_BETA * v + /*M_EPS**/input(t) + M_S_0)/M_TAU_R )
#define M_VEQ(v, y)((- v + y )/M_TAU_A)

//----- Hopf
#define H_XEQ(x, y, xother, theta, input, eps) (H_MYU - (x * x + y *y )) * x - tanh(xother) - theta*y + /*eps **/ input(t)
#define H_YEQ(x, y, theta) (H_MYU - (x * x + y * y)) * y + theta * x
#define H_THETAEQ(x, y, input, eta) -eta * input(t) * y / sqrt(x * x + y * y)

double r(double x, double y);
double PF(double Vi);
double S(double vmes);
double MN(double PFi, double Si);
double inputSignal(double t);
double inputSignal2(double t);
double theta(double x, double y);

#endif /* common_h */
