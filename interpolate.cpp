#include "config.h"
#include <iostream>
#include <vector>

using namespace std;

double interpolateThrust(const motorConfig& motor, double t) {
    if (motor.time_thrust_mass.rows() == 0) return 0.0;
    
    if (t <= motor.time_thrust_mass(0, 0)) {
        return motor.time_thrust_mass(0, 1);
    }
    if (t >= motor.time_thrust_mass(motor.time_thrust_mass.rows()-1, 0)) {
        return 0.0;
    }
    
    for(int i = 1; i < motor.time_thrust_mass.rows(); i++){
        if(t == motor.time_thrust_mass(i, 0)){
            return motor.time_thrust_mass(i, 1);
        }
        else if((t > motor.time_thrust_mass(i-1, 0)) && (t < motor.time_thrust_mass(i, 0))){
            double t_prev = motor.time_thrust_mass(i-1, 0);
            double t_next = motor.time_thrust_mass(i, 0);
            double thrust_prev = motor.time_thrust_mass(i-1, 1);
            double thrust_next = motor.time_thrust_mass(i, 1);

            double dt = t_next - t_prev;
            double alpha = (t - t_prev) / dt;
            return thrust_prev + alpha * (thrust_next - thrust_prev);
        }
    }

    /*for (int i = 1; i < motor.time_thrust_mass.rows(); i++) {
        if (t <= motor.time_thrust_mass(i, 0)) {
            double t_prev = motor.time_thrust_mass(i-1, 0);
            double t_next = motor.time_thrust_mass(i, 0);
            double thrust_prev = motor.time_thrust_mass(i-1, 1);
            double thrust_next = motor.time_thrust_mass(i, 1);
            
            double dt = t_next - t_prev;
            double alpha = (t - t_prev) / dt;
            return thrust_prev + alpha * (thrust_next - thrust_prev);
        }
    }*/
    return 0.0;
}

double interpolatePropellantMass(const motorConfig& motor, double t) {
    if (motor.time_thrust_mass.rows() == 0) return 0.0;
    
    if (t <= motor.time_thrust_mass(0, 0)) {
        return motor.time_thrust_mass(0, 2);
    }
    if (t >= motor.time_thrust_mass(motor.time_thrust_mass.rows()-1, 0)) {
        return motor.time_thrust_mass(motor.time_thrust_mass.rows()-1, 2);
    }

    for(int i = 1; i < motor.time_thrust_mass.rows(); i++){
        if(t == motor.time_thrust_mass(i, 0)){
            return motor.time_thrust_mass(i, 2);
        }
        else if((t > motor.time_thrust_mass(i-1, 0)) && (t < motor.time_thrust_mass(i, 0))){
            double t_prev = motor.time_thrust_mass(i-1, 0);
            double t_next = motor.time_thrust_mass(i, 0);
            double mass_prev = motor.time_thrust_mass(i-1, 2);
            double mass_next = motor.time_thrust_mass(i, 2);

            double dt = t_next - t_prev;
            double alpha = (t - t_prev) / dt;
            return mass_prev + alpha * (mass_next - mass_prev);
        }
    }
    /*
    for (int i = 1; i < motor.time_thrust_mass.rows(); i++) {
        if (t <= motor.time_thrust_mass(i, 0)) {
            double t_prev = motor.time_thrust_mass(i-1, 0);
            double t_next = motor.time_thrust_mass(i, 0);
            double mass_prev = motor.time_thrust_mass(i-1, 2);
            double mass_next = motor.time_thrust_mass(i, 2);
            
            double dt = t_next - t_prev;
            double alpha = (t - t_prev) / dt;
            return mass_prev + alpha * (mass_next - mass_prev);
        }
    }*/
    return 0.0;
}

pair<double, double> interpolateMOI(const rocketConfig& config, double mass) {
    if(config.simData.rows() == 0) return {0.0, 0.0};

    //cout << config.simData(0, 1) << endl;
    if(mass >= config.simData(0, 1)) return {config.simData(0, 2), config.simData(0, 3)};

    for(int i = 1; i < config.simData.rows(); i++){
        if(mass == config.simData(i, 1)){
            return {config.simData(i, 2), config.simData(i, 3)};
        }else if((mass < config.simData(i-1, 1)) && (mass > config.simData(i, 1))){
            double mass_prev = config.simData(i-1, 1);
            double mass_next = config.simData(i, 1);

            double lMOI_prev = config.simData(i-1, 2);
            double lMOI_next = config.simData(i, 2);

            double rMOI_prev = config.simData(i-1, 3);
            double rMOI_next = config.simData(i, 3);

            double dMass = mass_next - mass_prev;
            double alpha = (mass - mass_prev) / dMass;
            
            
            return {lMOI_prev + alpha * (lMOI_next - lMOI_prev), rMOI_prev + alpha * (rMOI_next - rMOI_prev)};
        }
    }
    return{0.0, 0.0};
}

pair<double, double> interpolateCGCP(const rocketConfig& config, double mass, double velocity) {
    double CG_position = 0.0;
    double CP_position = 0.0;

    if(config.simData.rows() == 0) return {0.0, 0.0};
    if((velocity <= config.simData(0, 0) || (mass >= config.simData(0, 1)))) return {config.simData(0, 4), config.simData(0, 5)};


    bool foundCP = false;
    for(int i = 1; i < config.simData.rows(); i++){
        if(velocity == config.simData(i, 0)){
            CP_position = config.simData(i, 5);
            foundCP = true;
            break;  // Only breaks the CP loop now
        }
        else if((velocity > config.simData(i-1, 0)) && (velocity < config.simData(i, 0))){
            // Linear interpolation
            double velocity_prev = config.simData(i-1, 0);
            double velocity_next = config.simData(i, 0);
            double CP_prev = config.simData(i-1, 5);
            double CP_next = config.simData(i, 5);

            double dVelocity = velocity_next - velocity_prev;
            double alpha = (velocity - velocity_prev) / dVelocity;
            CP_position = CP_prev + alpha * (CP_next - CP_prev);
            foundCP = true;
            break;
        }
    }
    
    // If no CP found, use last value
    if(!foundCP) {
        CP_position = config.simData(config.simData.rows()-1, 5);
    }

    // Find CG based on mass (column 1 = mass, column 4 = CG)
    bool foundCG = false;
    for(int i = 1; i < config.simData.rows(); i++){
        if(mass == config.simData(i, 1)){
            CG_position = config.simData(i, 4);
            foundCG = true;
            break;  // Only breaks the CG loop now
        }
        else if((mass > config.simData(i-1, 1)) && (mass < config.simData(i, 1))){
            // Linear interpolation
            double mass_prev = config.simData(i-1, 1);
            double mass_next = config.simData(i, 1);
            double CG_prev = config.simData(i-1, 4);
            double CG_next = config.simData(i, 4);

            double dMass = mass_next - mass_prev;
            double alpha = (mass - mass_prev) / dMass;
            CG_position = CG_prev + alpha * (CG_next - CG_prev);
            foundCG = true;
            break;
        }
    }
    
    // If no CG found, use last value
    if(!foundCG) {
        CG_position = config.simData(config.simData.rows()-1, 4);
    }

    return {CG_position, CP_position};
}