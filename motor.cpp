#include "Motor.h"
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

// Private static helper to extract an atribute
string Motor::extractAttribute_(const string& line, const string& attribute) {
    size_t pos = line.find(attribute + "=\"");
    if (pos == string::npos) return "";
    
    pos += attribute.length() + 2; // Move past 'attribute="'
    size_t end = line.find("\"", pos);
    if (end == string::npos) return "";
    return line.substr(pos, end - pos);
}

//File Loading 
bool Motor::loadFromFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Could not open file: " << filename << endl;
        return false;
    }

    vector<double> time, thrust, mass; 
    string line;
    bool in_data_section = false;

    while (getline(file, line)) {
        // Parse engine header attributes
        if (line.find("<engine") != string::npos && line.find("mfg=") != string::npos) {
            // The <engine> tag might span multiple lines
            // Keep reading until we find the closing '>'
            string engineTag = line;
            while (engineTag.find(">") == string::npos) {
                string nextLine;
                if (!getline(file, nextLine)) break;
                engineTag += " " + nextLine;  // Concatenate lines
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

    table_.resize(time.size(), 3);

    // Fill matrix
    for (int i = 0; i < time.size(); i++) {
        table_(i, 0) = time[i];
        table_(i, 1) = thrust[i];
        table_(i, 2) = mass[i];
    }
    
    // Verify we got data
    if (time.empty()) {
        cout << "Error: No thrust curve data found in file" << endl;
        return false;
    }
    
    return true;
}

// interpolation helpers

/**
 * Linearly interpolates column `col` of the table at time t.
 * `endValue` is returned once t exceeds the last entry.
 *
 * Linear interpolation works like this:
 *   Given two known points (t0, v0) and (t1, v1),
 *   the value at t is:  v = v0 + (t-t0)/(t1-t0) * (v1-v0)
 *   The fraction (t-t0)/(t1-t0) is called alpha (0 = at t0, 1 = at t1).
 */
double Motor::interpolate_(double t, int col, double endValue) const {
    if (table_.rows() == 0) return 0.0;

    // Before the first entry — return first value
    if (t <= table_(0, 0)) return table_(0, col);

    // Past the last entry — return the specified end value
    if (t >= table_(table_.rows() - 1, 0)) return endValue;

    // Find the bracketing rows and interpolate
    for (int i = 1; i < table_.rows(); i++) {
        double t0 = table_(i-1, 0), t1 = table_(i, 0);
        if (t >= t0 && t <= t1) {
            double alpha = (t - t0) / (t1 - t0);
            return table_(i-1, col) + alpha * (table_(i, col) - table_(i-1, col));
        }
    }
    return endValue;
}

double Motor::thrustAt(double t) const {
    // Thrust goes to 0 after burn-out
    return interpolate(t, 1, 0.0);
}

double Motor::propMassAt(double t) const {
    // Propellant mass stays at last recorded value after burn-out
    return interpolate(t, 2, table_(table_.rows()-1, 2));
}