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

// Log current state to flight data
void logData() {
    atmosphere atm = getAtmosphere(state.position[2]);
    Vector3d attitudeEuler = state.attitude.toRotationMatrix().eulerAngles(2, 1, 0);
    double speed = state.velocity.norm();
    double mach = speed / atm.speedOfSound;
    double q = 0.5 * atm.density * speed * speed;
    
    data.time.push_back(state.time);
    data.positionX.push_back(state.position[0]);
    data.positionY.push_back(state.position[1]);
    data.positionZ.push_back(state.position[2]);
    data.velocity_magnitude.push_back(speed);
    data.vertical_velocity.push_back(state.velocity[2]);
    data.horizontal_velocity.push_back(sqrt(state.velocity[0]*state.velocity[0] + state.velocity[1]*state.velocity[1]));
    
    double thrustForce = interpolateThrust(motor, state.time);
    double propellantMass = interpolatePropellantMass(motor, state.time);
    auto [aeroForce, aeroMoment] = calculateAerodynamics(state);
    
    data.acceleration_magnitude.push_back((aeroForce.norm() + thrustForce) / state.mass);
    data.roll.push_back(attitudeEuler[2] * 180.0 / M_PI);
    data.pitch.push_back(attitudeEuler[1] * 180.0 / M_PI);
    data.yaw.push_back(attitudeEuler[0] * 180.0 / M_PI);
    data.roll_rate.push_back(state.angularVelocity[0] * 180.0 / M_PI);
    data.pitch_rate.push_back(state.angularVelocity[1] * 180.0 / M_PI);
    data.yaw_rate.push_back(state.angularVelocity[2] * 180.0 / M_PI);
    data.thrust.push_back(thrustForce);
    data.drag.push_back(aeroForce.norm());
    data.mass.push_back(state.mass);
    data.req_roll_rate.push_back(PID_state.roll_rate);
    data.req_pitch_rate.push_back(PID_state.pitch_rate);
    data.req_yaw_rate.push_back(PID_state.yaw_rate);
    data.req_roll_cont.push_back(PID_state.roll_controll);
    data.req_pitch_cont.push_back(PID_state.pitch_controll);
    data.req_yaw_cont.push_back(PID_state.yaw_controll);
    data.fin1.push_back(state.finDeflections[0]);
    data.fin2.push_back(state.finDeflections[1]);
    data.fin3.push_back(state.finDeflections[2]);
    data.fin4.push_back(state.finDeflections[3]);
    data.mach_number.push_back(mach);
    data.dynamic_pressure.push_back(q);
}

void setInitialConditions(double launch_angle = 0.0, Vector3d initial_perturbation = Vector3d::Zero(), Vector3d inital_omega = Vector3d::Zero()) {
    state.position = Vector3d(0, 0, 0); // Launch from ground level
    state.velocity = Vector3d(0, 0, 0);
    state.attitude = eulerToQuaternion(Vector3d(initial_perturbation[0], launch_angle + initial_perturbation[1], initial_perturbation[2]));
    state.angularVelocity = Vector3d(inital_omega[0], inital_omega[1], inital_omega[2]);
    double motorDryMass = motor.initialWeight - motor.propellantWeight;
    double propellantMass = motor.propellantWeight;
    state.mass = config.massDry + motorDryMass + propellantMass;
    state.finDeflections = Vector4d::Zero();
    state.finCommands = Vector4d::Zero();
    state.time = 0.0;
    casOnline = false;
    
    // Clear previous data
    data = flightData();
}

bool saveDataToCSV(const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cout << "Error: Could not create output file: " << filename << endl;
        return false;
    }
    
    // Write header
    file << "time,positionX,positionY,positionZ,velocity_magnitude,vertical_velocity,horizontal_velocity,"
            << "acceleration_magnitude, roll,pitch,yaw, roll_rate,pitch_rate,yaw_rate,"
            << "thrust,drag,mass,req_roll_rate,req_pitch_rate,req_yaw_rate,req_roll_cont,req_pitch_cont,req_yaw_cont,fin1,fin2,fin3,fin4,"
            << "mach_number,dynamic_pressure" << endl;
    
    // Write data
    for (size_t i = 0; i < data.time.size(); i++) {
        file << fixed << setprecision(6)
                << data.time[i] << ","
                << data.positionX[i] << ","
                << data.positionY[i] << ","
                << data.positionZ[i] << ","
                << data.velocity_magnitude[i] << ","
                << data.vertical_velocity[i] << ","
                << data.horizontal_velocity[i] << ","
                << data.acceleration_magnitude[i] << ","
                << data.roll[i] << ","
                << data.pitch[i] << ","
                << data.yaw[i] << ","
                << data.roll_rate[i] << ","
                << data.pitch_rate[i] << ","
                << data.yaw_rate[i] << ","
                << data.thrust[i] << ","
                << data.drag[i] << ","
                << data.mass[i] << ","
                << data.req_roll_rate[i] << ","
                << data.req_pitch_rate[i] << ","
                << data.req_yaw_rate[i] << ","
                << data.req_roll_cont[i] << ","
                << data.req_pitch_cont[i] << ","
                << data.req_yaw_cont[i] << ","
                << data.fin1[i] << ","
                << data.fin2[i] << ","
                << data.fin3[i] << ","
                << data.fin4[i] << ","
                << data.mach_number[i] << ","
                << data.dynamic_pressure[i] << endl;
    }
    
    file.close();
    cout << "Flight data saved to: " << filename << endl;
    return true;
}