#pragma once

#include <iostream>
#include <vector>
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include <fstream>
#include <unordered_map>
#include <cmath>
#include <ctime>
#include <mutex>
#include <functional>
#include <random>
#include <iomanip>
#include <string>

using namespace std;
using namespace Eigen;

// rocket configuration sructure
struct rocketConfig{
    // rocket physical properties
    double massDry = 4.57;            // kg - rocket dry mass
    double length = 2;            // m  - rocket length
    double diameter = 0.104;        // m  - rocket OD
    double Cn_alpha = 11.065;        // Normal force coefficient due to angle of attack (body + fins)

    // data from rocket sim (kg*m^2)
    Matrix<double, Dynamic, 6> simData; 

    // rocket aerodynamic properties
    double cd = 0.406;               // coeficient of drag
    double referenceArea = 0.00849;  // m^2 - cross-sectional area

    // control fin configuration (assumes four fins always)
    double span = 0.038;
    double rootChord = 0.089;
    double tipChord = 0.025;
    double finArea = 0.5 * (rootChord + tipChord) * span;          // m^2 - area of each fin
    double finAspectRatio = (span * span) / finArea;      // aspect ratio of fins (may be unused)
    double finLocation = 0.3;       // m - distance of fins from nose
    double maxFinDeflection = 10;   // degrees - maximum fin deflection
};
// motor configuration
struct motorConfig{
    double initialWeight;                // kg
    double propellantWeight;             // kg
    double isp;                         // seconds
    
    Matrix<double, Dynamic, 3> time_thrust_mass;          // seconds_newtons_grams
};
// model for atmosphere
class Atmosphere{
public:
    double density;                 // kg/m^3
    double pressure;                // Pa (unused?)
    double temperature;             // K (unused?)
    double speedOfSound;            // m/s

    // functions
    
}
// current thrust point (CHANGE TO BE VECTOR SO IT DOESNT UPDATE EVERY TIME?)
struct thrustPoint{
    double time;                    // seconds
    double thrust;                  // newtons
    double massFlow;                // kg/s
};
// rocket current state vector
struct rocketState{
    Vector3d position = Vector3d::Zero();                   // m     - position
    Vector3d velocity = Vector3d::Zero();                   // m/s   - velocity
    Quaterniond attitude = Quaterniond::Identity();                   // rad   - quaternian
    Vector3d angularVelocity = Vector3d::Zero();            // rad/s - angular velocity (BODY FRAME)
    Quaterniond desiredAttitude = Quaterniond::Identity();

    double mass;                                            // kg    - current mass
    double time;
    int step = 0;

    Vector4d finDeflections = Vector4d::Zero();              // degrees
    Vector4d finCommands = Vector4d::Zero();                // degrees
};
// Flight data logging
struct flightData {
    vector<double> time;
    vector<double> positionX;
    vector<double> positionY;
    vector<double> positionZ;
    vector<double> velocity_magnitude;
    vector<double> vertical_velocity;
    vector<double> horizontal_velocity;
    vector<double> acceleration_magnitude;
    vector<double> roll, pitch, yaw;
    vector<double> roll_rate, pitch_rate, yaw_rate;
    vector<double> thrust;
    vector<double> drag;
    vector<double> mass;
    vector<double> req_roll_rate, req_pitch_rate, req_yaw_rate;
    vector<double> req_roll_cont, req_pitch_cont, req_yaw_cont;
    vector<double> fin1, fin2, fin3, fin4;
    vector<double> mach_number;
    vector<double> dynamic_pressure;
};

struct stateDerivative {
    Vector3d velocity;           // Rate of change of position
    Vector3d acceleration;       // Rate of change of velocity
    Quaterniond quaternionDerivative;    // Rate of change of attitude
    Vector3d angularAcceleration; // Rate of change of angular velocity
};