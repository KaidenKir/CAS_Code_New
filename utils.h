#pragma once

#include <iostream>l
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include <string>

using namespace std;
using namespace Eigen;

void logData();

void setInitialConditions(double launch_angle = 0.0, Vector3d initial_perturbation = Vector3d::Zero(), Vector3d inital_omega = Vector3d::Zero());

bool saveDataToCSV(const string& filename);
