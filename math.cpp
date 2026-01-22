#include "math.h"
#include "interpolate.h"
#include "config.h"

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

double sign(double x){
    if(x > 0) return 1.0;
    if(x < 0) return -1.0;
    else return 0.0;
}

bool checkIfFinite(double var, string varName){
    if(std::isnan(var)){
        cout << varName << " is NaN" << endl;
        return true;
    }else if(std::isinf(var)){
        cout << varName << " is Infinity" << endl;
    }
    return false;
}

bool checkIfFinite(Vector3d var, string varName){
    if(var.hasNaN()){
        cout << varName << " has NaN" << endl;
        return true;
    }else if(!var.allFinite()){
        cout << varName << " has Infinity" << endl;
    }
    return false;
}

// quaternian helper functions;
// this vector computes the quaternian to axis angles
Vector3d quaternionToAngles(const Quaterniond& q){
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();

    // quaternian encodes rotation as q(w, x, y, z) = (cos(theta/2), nx*sin(theta/2), ny*sin(theta/2), nz*sin(theta/2))
    // to calculate theta, w = cos(theta/2) => theta = 2*acos(w) => clamp w to protect against greater or less than 1
    double theta = 2*acos(clamp(w, -1.0, 1.0));

    // we need to calculate the magnitude of the vector to find the direction of the returned angle vector
    // since (nx*sin(theta/2), ny*sin(theta/2), nz*sin(theta/2)) = (x, y, z) => (nx*sin(theta/2) + ny*sin(theta/2) + nz*sin(theta/2)) = x + y + z
    // square both sides and pull out sin(theta/2): sin^2(theta/2)*(a unit vector) = x^2 + y^2 + z^2 => sin^2(theta/2) = x^2 + y^2 + z^2
    // sin(theta/2) = magnitude of the vector
    double sin_half_theta = sqrt(x*x + y*y + z*z);

    // axis-angle is represented as theta * n
    // n = (x, y, z)/sin(theta/2)
    // axis-angle = (theta/sin(theta/2))*(x, y, z)
    Vector3d axis_angle = (theta/sin_half_theta) * Vector3d(x, y, z);

    // if theta is small, sin goes to 0, which means /0. we use the limit of theta/sin(theta/2) = 2 to fix this
    if(theta < 1e-6){
        axis_angle = 2*Vector3d(x, y, z);
    }
    return axis_angle;
}

// euler vector to Quaternion
Quaterniond eulerToQuaternion(const Vector3d& vector){
    double roll = vector[0];
    double pitch = vector[1];
    double yaw = vector[2];
    return AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll, Vector3d::UnitX());
}

// creates a matrix to rotate from body to world
Matrix3d rotationalMatrix(const Quaterniond& q){
    return q.normalized().toRotationMatrix();
}

// rotates a vector by a quaternion
Vector3d rotate_vector_by_quaternion(Vector3d vec, Quaterniond q){
    // get vector part of quaternion
    Vector3d u(q.x(), q.y(), q.z());

    // get scalar part of quaternion
    double s = q.w();

    // rotation math
    return 2.0 * u.dot(vec) * u + (s*s - u.dot(u)) * vec + 2.0 * s * u.cross(vec);
}

// multiplys a quaternion by a scaler
Quaterniond multByScalar(const Quaterniond& q, double scalar){
    return Quaterniond(
                        q.w() * scalar,
                        q.x() * scalar,
                        q.y() * scalar,
                        q.z() * scalar
    );
}

void integrate_RK4(double dt, double thrustForce) {
    // Update mass for current time
    double propellantMass = interpolatePropellantMass(motor, state.time);
    double motorDryMass = motor.initialWeight - motor.propellantWeight;
    state.mass = config.massDry + motorDryMass + propellantMass;
    
    // RK4 stages
    // k1 = f(t, y)
    stateDerivative k1 = calculateDerivatives(state, state.time, thrustForce);
    if(k1.acceleration.hasNaN() || k1.angularAcceleration.hasNaN()) {
        cout << "ERROR in k1 at time " << state.time << endl;
        cout << "Acceleration: " << k1.acceleration.transpose() << endl;
        cout << "Angular accel: " << k1.angularAcceleration.transpose() << endl;
        cout << "Attitude: " << state.attitude.w() << ", " << state.attitude.x() << ", " 
            << state.attitude.y() << ", " << state.attitude.z() << endl;
        cout << "Angular velocity: " << state.angularVelocity.transpose() << endl;
    }
    
    // k2 = f(t + dt/2, y + k1*dt/2)
    rocketState state2 = addScaledDerivative(state, k1, dt/2.0);
    state2.time = state.time + dt/2.0;
    stateDerivative k2 = calculateDerivatives(state2, state2.time, thrustForce);
    if(k2.acceleration.hasNaN() || k2.angularAcceleration.hasNaN()) {
        cout << "ERROR in k2 at time " << state.time << endl;
        cout << "Acceleration: " << k2.acceleration.transpose() << endl;
        cout << "Angular accel: " << k2.angularAcceleration.transpose() << endl;
        cout << "Attitude: " << state.attitude.w() << ", " << state.attitude.x() << ", " 
            << state.attitude.y() << ", " << state.attitude.z() << endl;
        cout << "Angular velocity: " << state.angularVelocity.transpose() << endl;
    }
    
    // k3 = f(t + dt/2, y + k2*dt/2)
    rocketState state3 = addScaledDerivative(state, k2, dt/2.0);
    state3.time = state.time + dt/2.0;
    stateDerivative k3 = calculateDerivatives(state3, state3.time, thrustForce);
    if(k3.acceleration.hasNaN() || k3.angularAcceleration.hasNaN()) {
        cout << "ERROR in k3 at time " << state.time << endl;
        cout << "Acceleration: " << k3.acceleration.transpose() << endl;
        cout << "Angular accel: " << k3.angularAcceleration.transpose() << endl;
        cout << "Attitude: " << state.attitude.w() << ", " << state.attitude.x() << ", " 
            << state.attitude.y() << ", " << state.attitude.z() << endl;
        cout << "Angular velocity: " << state.angularVelocity.transpose() << endl;
    }
    
    // k4 = f(t + dt, y + k3*dt)
    rocketState state4 = addScaledDerivative(state, k3, dt);
    state4.time = state.time + dt;
    stateDerivative k4 = calculateDerivatives(state4, state4.time, thrustForce);
    if(k4.acceleration.hasNaN() || k4.angularAcceleration.hasNaN()) {
        cout << "ERROR in k4 at time " << state.time << endl;
        cout << "Acceleration: " << k4.acceleration.transpose() << endl;
        cout << "Angular accel: " << k4.angularAcceleration.transpose() << endl;
        cout << "Attitude: " << state.attitude.w() << ", " << state.attitude.x() << ", " 
            << state.attitude.y() << ", " << state.attitude.z() << endl;
        cout << "Angular velocity: " << state.angularVelocity.transpose() << endl;
    }
    
    // Combine: y_new = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    state.position += (dt/6.0) * (k1.velocity + 2*k2.velocity + 2*k3.velocity + k4.velocity);
    state.velocity += (dt/6.0) * (k1.acceleration + 2*k2.acceleration + 2*k3.acceleration + k4.acceleration);

    Quaterniond dq(
                        (dt/6.0) * (k1.quaternionDerivative.w() + 2*k2.quaternionDerivative.w() + 2*k3.quaternionDerivative.w() + k4.quaternionDerivative.w()), 
                        (dt/6.0) * (k1.quaternionDerivative.x() + 2*k2.quaternionDerivative.x() + 2*k3.quaternionDerivative.x() + k4.quaternionDerivative.x()), 
                        (dt/6.0) * (k1.quaternionDerivative.y() + 2*k2.quaternionDerivative.y() + 2*k3.quaternionDerivative.y() + k4.quaternionDerivative.y()), 
                        (dt/6.0) * (k1.quaternionDerivative.z() + 2*k2.quaternionDerivative.z() + 2*k3.quaternionDerivative.z() + k4.quaternionDerivative.z()) 
    );
    Quaterniond qNext(
                        state.attitude.w() + dq.w(),
                        state.attitude.x() + dq.x(),
                        state.attitude.y() + dq.y(),
                        state.attitude.z() + dq.z()
    );
    state.attitude = qNext;
    state.angularVelocity += (dt/6.0) * (k1.angularAcceleration + 2*k2.angularAcceleration + 2*k3.angularAcceleration + k4.angularAcceleration);
    if(state.angularVelocity.norm() > 500){
        cout << "k1: " << k1.angularAcceleration << " | k2:" << 2*k2.angularAcceleration << " | k3:" <<  2*k3.angularAcceleration << " | k4: " << k4.angularAcceleration << endl;
    }
    
    // Final angle wrapping
    state.attitude.normalize();
}

stateDerivative calculateDerivatives(const rocketState& tempState_Deriv, double time, double thrustForce) {
    stateDerivative deriv;
    
    // Calculate mass at this specific time point
    double propellantMass = interpolatePropellantMass(motor, time);
    double motorDryMass = motor.initialWeight - motor.propellantWeight;
    double currentMass = config.massDry + motorDryMass + propellantMass;

    // Get forces and moments at this state
    auto [aeroForce, aeroMomentBody] = calculateAerodynamics(tempState_Deriv);
    
    // Transform thrust from body to world frame
    Vector3d thrustBody(0.0, 0.0, thrustForce);
    Vector3d thrustWorld = tempState_Deriv.attitude._transformVector(thrustBody);
    
    // Gravity in world frame
    Vector3d gravity(0, 0, -9.81 * currentMass);
    
    // Total force
    Vector3d totalForce = thrustWorld + aeroForce + gravity;
    
    // TRANSLATIONAL DERIVATIVES
    deriv.velocity = tempState_Deriv.velocity;  // dx/dt = v-
    deriv.acceleration = totalForce / currentMass;  // dv/dt = F/m
    
    // ROTATIONAL DERIVATIVES
    // Update moments of inertia based on current mass
    auto [lMOI, rMOI] = interpolateMOI(config, currentMass);
    Matrix3d I = Matrix3d::Zero();
    I(0,0) = rMOI;
    I(1,1) = rMOI;
    I(2,2) = lMOI;

    Vector3d  angularVelocityBody = tempState_Deriv.angularVelocity;

    // eulers rotation equation in body frame
    // includes gyroscopic effects 
    // d(omega)/dt = I^-1 * (M - omega X (I*omega))
    Vector3d gyroscopicTerm = angularVelocityBody.cross(I * angularVelocityBody);
    Vector3d angularAccelBody = I.inverse() * (aeroMomentBody - gyroscopicTerm);
    if(angularAccelBody.hasNaN()){
        cout << "I: " << I << endl;
        cout << "I.inverse(): " << I.inverse() << endl;
        cout << "aeroMomentBody: " << aeroMomentBody << endl;
        cout << "gyroscopicTerm: " << gyroscopicTerm << endl;
    }

    deriv.angularAcceleration = angularAccelBody;

    // quaternion derivative
    Quaterniond q = tempState_Deriv.attitude.normalized();

    //angular velocity as pure quaternion (0, omega)
    Quaterniond omegaQuat(0, angularVelocityBody.x(), angularVelocityBody.y(), angularVelocityBody.z());
    deriv.quaternionDerivative = (multByScalar((q * omegaQuat), 0.5)); 

    return deriv;
}

rocketState addScaledDerivative(const rocketState& tempState_ScaleDeriv, const stateDerivative& deriv, double scale) {
    rocketState newState = tempState_ScaleDeriv;
    
    newState.position += deriv.velocity * scale;
    newState.velocity += deriv.acceleration * scale;
    Quaterniond toAdd = multByScalar(deriv.quaternionDerivative, scale);
    newState.attitude = Quaterniond(
                            newState.attitude.w() + toAdd.w(),
                            newState.attitude.x() + toAdd.x(),
                            newState.attitude.y() + toAdd.y(),
                            newState.attitude.z() + toAdd.z()
    );
    newState.angularVelocity += deriv.angularAcceleration * scale;
    
    newState.attitude.normalize();
    
    return newState;
}