#pragma once

#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include "MathUtils.h"

using namespace Eigen;

class RocketState{
public:
    // Rocket State Items
    Vector3d position        = Vector3d::Zero(); //m, WORLD FRAME
    Vector3d velocity        = Vector3d::Zero(); //m, WORLD FRAME
    Quaternion attitude      = Quaterniond(0, 0, 0, 1); // body -> world
    Vector3d angularVelocity = Vector3d::Zero(); //rad/s, BODY FRAME

    // Rocket Guidance Items
    Quaterniond desiredAttitude = Quaternion(0, 0, 0, 1);
    Vector4d finDeflections     = Vector4d::Zero(); //degrees, (+Y, +X, -Y, -X)
    Vector4d finCommands        = Vector4d::Zero(); //degrees (comands for logging)

    // Misk Items
    double mass = 0.0; //kg, rockets current mass
    double time = 0.0; //s, simulation clock
    int step    = 0;   //-, simulation step counter

    /**resets to fresh rocket state */
    void reset();
}

/**
 * StateDerivative
 * ---------------
 * The time derivative of a RocketState, as returned by the equations of motion.
 * Used by the RK4 integrator:  state_new = state + derivative * dt
 */
struct StateDerivative {
    Vector3d    velocity;              // d(position)/dt
    Vector3d    acceleration;          // d(velocity)/dt
    Quaterniond quaternionDerivative;  // d(attitude)/dt   (NOT a unit quaternion)
    Vector3d    angularAcceleration;   // d(angularVelocity)/dt
};