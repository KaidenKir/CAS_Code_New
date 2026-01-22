#pragma once

#include "config.h"
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"

using namespace std;
using namespace Eigen;

double sign(double x);

bool checkIfFinite(double var, string varName);

bool checkIfFinite(Vector3d var, string varName);

Vector3d quaternionToAngles(const Quaterniond& q);

Quaterniond eulerToQuaternion(const Vector3d& vector);

Matrix3d rotationalMatrix(const Quaterniond& q);

Vector3d rotate_vector_by_quaternion(Vector3d vec, Quaterniond q);

Quaterniond multByScalar(const Quaterniond& q, double scalar);

void integrate_RK4(double dt, double thrustForce);

stateDerivative calculateDerivatives(const rocketState& tempState_Deriv, double time, double thrustForce);

rocketState addScaledDerivative(const rocketState& tempState_ScaleDeriv, const stateDerivative& deriv, double scale);

