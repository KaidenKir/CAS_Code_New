#include "fileManip.h"
#include "config.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

using namespace std;

string extractAttribute(const string& line, const string& attribute) {
    size_t pos = line.find(attribute + "=\"");
    if (pos == string::npos) return "";
    
    pos += attribute.length() + 2; // Move past 'attribute="'
    size_t end = line.find("\"", pos);
    if (end == string::npos) return "";
    
    return line.substr(pos, end - pos);
}

bool parseRSEFile(const string& filename, motorConfig& motor){
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Could not open file: " << filename << endl;
        return false;
    }
    
    string line;
    bool in_data_section = false;

    vector<double> time;
    vector<double> thrust;
    vector<double> mass;    
    while (getline(file, line)) {
        // Parse engine header attributes
        if (line.find("<engine") != string::npos && line.find("mfg=") != string::npos) {
            // The <engine> tag might span multiple lines
            // Keep reading until we find the closing '>'
            string engineTag = line;
            while (engineTag.find(">") == string::npos) {
                string nextLine;
                if (getline(file, nextLine)) {
                    engineTag += " " + nextLine;  // Concatenate lines
                } else {
                    break;
                }
            }
            
            // Now extract attributes from the complete tag
            string initWt = extractAttribute(engineTag, "initWt");
            if (!initWt.empty()) motor.initialWeight = stod(initWt)/1000;
            
            string propWt = extractAttribute(engineTag, "propWt");
            if (!propWt.empty()) motor.propellantWeight = stod(propWt)/1000;
            
            string isp = extractAttribute(engineTag, "Isp");
            if (!isp.empty()) motor.isp = stod(isp);
        }
        
        // Check if entering data section
        if (line.find("<data>") != string::npos) {
            in_data_section = true;
            continue;
        }
        
        // Check if leaving data section
        if (line.find("</data>") != string::npos) {
            in_data_section = false;
            continue;
        }
        
        // Parse data points
        if (in_data_section && line.find("<eng-data") != string::npos) {
            string timeStr = extractAttribute(line, "t");
            string thrustStr = extractAttribute(line, "f");
            string massStr = extractAttribute(line, "m");
            
            if (!timeStr.empty() && !thrustStr.empty() && !massStr.empty()) {
                time.push_back(stod(timeStr));
                thrust.push_back(stod(thrustStr));
                mass.push_back(stod(massStr)/1000);
            }
        }
    }
    
    file.close();

    motor.time_thrust_mass.resize(time.size(), 3);

    // Fill matrix
    for (int i = 0; i < time.size(); i++) {
        motor.time_thrust_mass(i, 0) = time[i];
        motor.time_thrust_mass(i, 1) = thrust[i];
        motor.time_thrust_mass(i, 2) = mass[i];
    }
    
    // Verify we got data
    if (time.empty()) {
        cout << "Error: No thrust curve data found in file" << endl;
        return false;
    }
    
    return true;
}

bool parseRocketFile(const string& filename, rocketConfig& config){
     ifstream file(filename);

    if (!file.is_open()) {
        cout << "Error: Could not open file: " << filename << endl;
        return false;
    }

    string line;
    vector<double> velocity;
    vector<double> mass;
    vector<double> lMOI;
    vector<double> rMOI;    
    vector<double> CP_location;    
    vector<double> CG_location;    
    while (getline(file, line)) {
        // Check if entering data section
        if (line.empty() || line[0] == '#') {
            continue;
        }

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
        cout << "Error: No MOI data found in file" << endl;
        return false;
    }

    vector<bool> indicesToKeep;
    int countToKeep = 0;

    // First row is always kept (unless it has NaN CP)
    if(!std::isnan(CP_location[0])){
        indicesToKeep.push_back(1);
        countToKeep++;
    } else {
        indicesToKeep.push_back(0);
    }

    for(int i = 1; i < mass.size(); i++){  
        if((mass[i] == mass[i-1]) || (std::isnan(CP_location[i]))){
            indicesToKeep.push_back(0);
        } else{
            indicesToKeep.push_back(1);
            countToKeep++;
        }
        
    }

    config.simData.conservativeResize(countToKeep, 6);
    int index = 0;
    for(int i = 0; i < mass.size(); i++){
        if(indicesToKeep[i] == true){    
            config.simData.row(index) << velocity[i], mass[i], lMOI[i], rMOI[i], CG_location[i], CP_location[i];
            index++;
            //cout << config.simData.row(i) << endl;
        }
    }

    return true;
}