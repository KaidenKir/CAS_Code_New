#include "RocketConfig.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

bool RocketConfig::loadSimData(const string& filename){
    ifstream file(filename);

    if (!file.is_open()) {
        cout << "RocketConfig: Could not open file: " << filename << endl;
        return false;
    }

    string line;
    vector<double> velocity, mass, lMOI, rMOI, CP_location, CG_location; 

    while (getline(file, line)) {
        // Check if entering data section
        if (line.empty() || line[0] == '#') continue;

        stringstream ss(line);
        string token;
        vector<double> values;

        while(getline(ss, token, ',')){
            try{
                values.push_back(stod(token));
            } catch (const exception& e){
                cout << "Could not parse value: " << token << endl;
                continue;
            }
        }

        if(values.size() == 6){
            velocity.push_back(values[0]);
            mass.push_back(values[1]);
            lMOI.push_back(values[2]);
            rMOI.push_back(values[3]);
            CP_location.push_back(-values[4]/1000);       
            CG_location.push_back(-values[5]/1000);
        }
    }
    file.close();

    if (mass.empty()) {
        cout << "RocketConfig: No MOI data found in file" << endl;
        return false;
    }

    // Remove duplicate / invalid rows 
    // Rows where mass didn't change or CP is NaN are noise from the sim export.
    vector<bool> indicesToKeep;
    int countToKeep = 0;
    for(int i = 1; i < mass.size(); i++){  
        if((mass[i] == mass[i-1]) || (std::isnan(CP_location[i-1]))){
             indicesToKeep.push_back(0);
        } else{
            indicesToKeep.push_back(1);
            countToKeep++;
        }
        
    }

    simData_.conservativeResize(countToKeep, 6);
    int index = 0;
    for(int i = 0; i < mass.size()-1; i++){
        if(indicesToKeep[i] == true){    
            simData_.row(index) << velocity[i], mass[i], lMOI[i], rMOI[i], CG_location[i], CP_location[i];
            index++;
            //cout << simData_.row(i) << endl;
        }
    }

    return true;
}

/**
 * The sim table is sorted by decreasing mass (propellant burns off).
 * We search for the two rows that bracket the current mass and linearly
 * interpolate the MOI values between them.
 *
 * col 1 = mass, col 2 = longitidinal MOI, col 3 = radial MOI
 */
pair<double, double> getMOI(double mass){
    if(simData_.rows() == 0) return {0.0, 0.0};

    if(mass <= simData_(0, 1)) return {simData_(0, 2), simData_(0, 3)};

    for(int i = 1; i < simData_.rows(); i++){
        if(mass == simData_(i, 1)){
            return {simData_(i, 2), simData_(i, 3)};
        }else if(mass > simData_(0, 1)){
            return{simData_(0, 2), simData_(0, 3)};
        }else if((mass > simData_(i-1, 1)) && (mass < simData_(i, 1))){
            double mass_prev = simData_(i-1, 1);
            double mass_next = simData_(i, 1);

            double lMOI_prev = simData_(i-1, 2);
            double lMOI_next = simData_(i, 2);

            double rMOI_prev = simData_(i-1, 3);
            double rMOI_next = simData_(i, 3);

            double dMass = mass_next - mass_prev;
            double alpha = (mass - mass_prev) / dMass;
            
            
            return {lMOI_prev + alpha * (lMOI_next - lMOI_prev), rMOI_prev + alpha * (rMOI_next - rMOI_prev)};
        }
    }
    return{0.0, 0.0};
}

/**
 * CG depends on mass (propellant burn-off shifts the CG).
 * CP depends on velocity (compressibility moves the CP).
 * Each is found independently with its own linear search.
 */
pair<double, double> RocketConfig::getCGandCP(double mass, double velocity) const {
    double CG_position = 0.0;
    double CP_position = 0.0;

    if(simData_.rows() == 0) return {0.0, 0.0};
    if((velocity <= simData_(0, 0) || (mass >= simData_(0, 1)))) return {simData_(0, 4), simData_(0, 5)};


    bool foundCP = false;
    for(int i = 1; i < simData_.rows(); i++){
        if(velocity == simData_(i, 0)){
            CP_position = simData_(i, 5);
            foundCP = true;
            break;  // Only breaks the CP loop now
        }
        else if((velocity > simData_(i-1, 0)) && (velocity < simData_(i, 0))){
            // Linear interpolation
            double velocity_prev = simData_(i-1, 0);
            double velocity_next = simData_(i, 0);
            double CP_prev = simData_(i-1, 5);
            double CP_next = simData_(i, 5);

            double dVelocity = velocity_next - velocity_prev;
            double alpha = (velocity - velocity_prev) / dVelocity;
            CP_position = CP_prev + alpha * (CP_next - CP_prev);
            foundCP = true;
            break;
        }
    }
    
    // If no CP found, use last value
    if(!foundCP) {
        CP_position = simData_(simData_.rows()-1, 5);
    }

    // Find CG based on mass (column 1 = mass, column 4 = CG)
    bool foundCG = false;
    for(int i = 1; i < simData_.rows(); i++){
        if(mass == simData_(i, 1)){
            CG_position = simData_(i, 4);
            foundCG = true;
            break;  // Only breaks the CG loop now
        }
        else if((mass > simData_(i-1, 1)) && (mass < simData_(i, 1))){
            // Linear interpolation
            double mass_prev = simData_(i-1, 1);
            double mass_next = simData_(i, 1);
            double CG_prev = simData_(i-1, 4);
            double CG_next = simData_(i, 4);

            double dMass = mass_next - mass_prev;
            double alpha = (mass - mass_prev) / dMass;
            CG_position = CG_prev + alpha * (CG_next - CG_prev);
            foundCG = true;
            break;
        }
    }
    
    // If no CG found, use last value
    if(!foundCG) {
        CG_position = simData_(simData_.rows()-1, 4);
    }

    return {CG_position, CP_position};
}