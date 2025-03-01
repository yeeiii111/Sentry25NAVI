#include "navigation/pid.hpp"

PID::PID(double p, double i, double d, double max_p, double max_i, double max_d, double dt):
    kp(p),ki(i),kd(d),max_p(max_p),max_i(max_i),max_d(max_d),dt(dt),integral(0){}

void PID::calculate(double dist)
{
    
}