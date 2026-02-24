#pragma once
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include "MathUtils.h"

using namespace Eigen;

/**
 * RocketState
 * -----------
 * Holds the complete state vector of the rocket at a single instant in time.
 *
 * Positions / velocities use the world (inertial) frame; Z is up.
 * Angular velocity is in the body frame (aligned with the rocket body axis).
 * Attitude is stored as a unit quaternion (body → world rotation).
 *
 * finDeflections stores the actual current fin angles in degrees.
 */
class RocketState {
public:
    // ── Translational ─────────────────────────────────────────────────────
    Vector3d position        = Vector3d::Zero();   // m      world frame
    Vector3d velocity        = Vector3d::Zero();   // m/s    world frame

    // ── Rotational ────────────────────────────────────────────────────────
    Quaterniond attitude     = Quaterniond(0, 0, 0, 1);  // body → world
    Vector3d angularVelocity = Vector3d::Zero();          // rad/s  body frame

    // ── Guidance ──────────────────────────────────────────────────────────
    Quaterniond desiredAttitude = Quaterniond(0, 0, 0, 1);

    // ── Actuators ─────────────────────────────────────────────────────────
    Vector4d finDeflections  = Vector4d::Zero();   // degrees  (+Y, +X, -Y, -X fins)
    Vector4d finCommands     = Vector4d::Zero();   // degrees  (commanded, for logging)

    // ── Bookkeeping ───────────────────────────────────────────────────────
    double mass = 0.0;   // kg   total current mass
    double time = 0.0;   // s    simulation clock
    int    step = 0;     // —    integer step counter

    /** Resets to a freshly constructed state, retaining nothing. */
    void reset();
};

// ─────────────────────────────────────────────────────────────────────────────

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
