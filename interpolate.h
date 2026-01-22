#pragma once

#include <vector>
using namespace std;

// interpolate between thrust points
double interpolateThrust(const motorConfig& motor, double t);

// interpolate between propelant mass points
double interpolatePropellantMass(const motorConfig& motor, double t);

// interpolate between MOI points
pair<double, double> interpolateMOI(const rocketConfig& config, double mass);

// interpolate between CGCP points
pair<double, double> interpolateCGCP(const rocketConfig& config, double mass, double velocity);