#pragma once

#include <vector>

using namespace std;

struct PID_vals{
    vector<double> PID_vals = {0.0, 0.0, 0.0};      // proportional, integral, derivative
    double integral = 0.0; 
    double prevActual = 0.0;
    double outputLimit = 5.0;
    double integralLimit = 10.0;
    bool initialized = false;

    const double tau = 0.02;
    double alpha = 0.0;
    double dFiltered = 0.0;
};

struct PID_state{
    double outer_dt = 0.02;
    double inner_dt = 0.002;
    double time_since_outer_update = 0.0;
    double time_since_inner_update = 0.0;

    double pitch_rate = 0;
    double yaw_rate = 0;
    double roll_rate = 0;

    double pitch_controll = 0;
    double yaw_controll = 0;
    double roll_controll = 0;
};

double PID(double actual, double goal, double dt, PID_vals& state);
