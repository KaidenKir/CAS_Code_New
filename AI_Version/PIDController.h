#pragma once
#include <vector>
#include "MathUtils.h"
#include "RocketState.h"

// ─────────────────────────────────────────────────────────────────────────────
/**
 * PIDController
 * -------------
 * A single-axis PID controller with:
 *   - Anti-windup: integral is cleared when its sign disagrees with the error
 *   - Integral clamping
 *   - Derivative low-pass filter (time constant tau)
 *   - Output clamping
 *
 * Call  update(actual, goal, dt)  each time-step to get the output.
 *
 * Example:
 *   PIDController pitchRate;
 *   pitchRate.setGains(2.0, 0.1, 0.5);    // Kp, Ki, Kd
 *   double cmd = pitchRate.update(measured, desired, 0.002);
 */
class PIDController {
public:
    // ── Tuning ───────────────────────────────────────────────────────────
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    double outputLimit   = 5.0;   // |output| is clamped to this
    double integralLimit = 10.0;  // |integral| is clamped to this
    double tau           = 0.02;  // derivative filter time constant (seconds)

    void setGains(double kp, double ki, double kd);

    /**
     * Advances the controller by one time-step.
     *
     * `actual` – the measured value of the quantity being controlled.
     * `goal`   – the set-point (desired value).
     * `dt`     – elapsed time since the last call (seconds).
     *
     * Returns the controller output (fin command / rate command, etc.)
     */
    double update(double actual, double goal, double dt);

    /** Clears internal state (integral accumulator, previous sample, etc.). */
    void reset();

private:
    double integral_    = 0.0;
    double prevActual_  = 0.0;
    double dFiltered_   = 0.0;
    double alpha_       = 0.0;   // filter coefficient  = dt / (tau + dt)
    bool   initialized_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────
/**
 * CASController  (Control Augmentation System)
 * --------------------------------------------
 * Implements a two-loop (cascade) attitude controller:
 *
 *   Outer loop  (runs at outerDt Hz)  — angle error → rate command
 *   Inner loop  (runs at innerDt Hz)  — rate error  → fin deflection
 *
 * The outer loop reads the error between the rocket's current quaternion and
 * the desired quaternion, converts it to an axis-angle error vector, and
 * feeds each axis through a PIDController to produce a rate command.
 *
 * The inner loop compares measured body rates to the commanded rates and
 * produces fin deflections.
 *
 * Fin assignment (four fins, body frame, Z = rocket axis):
 *   fin[0] = +Y fin  → controls pitch + roll
 *   fin[1] = +X fin  → controls yaw  + roll
 *   fin[2] = -Y fin  → controls pitch + roll (opposite sign)
 *   fin[3] = -X fin  → controls yaw  + roll (opposite sign)
 *
 * Usage:
 *   CASController cas;
 *   cas.setOuterGains(Kp, Ki, Kd);   // one set for all axes (can be extended)
 *   cas.setInnerGains(Kp, Ki, Kd);
 *   cas.update(state, dt, maxFinDeflection);  // modifies state.finDeflections
 */
class CASController {
public:
    double outerDt = 0.02;   // s  — outer loop update period
    double innerDt = 0.002;  // s  — inner loop update period

    // Outer loop controllers (one per body axis: pitch, yaw, roll)
    PIDController pitchOuter, yawOuter, rollOuter;

    // Inner loop controllers
    PIDController pitchInner, yawInner, rollInner;

    /** Convenience: set the same gains for all outer-loop axes. */
    void setOuterGains(double kp, double ki, double kd);

    /** Convenience: set the same gains for all inner-loop axes. */
    void setInnerGains(double kp, double ki, double kd);

    /**
     * Advances the CAS by dt seconds.
     * Reads state.attitude, state.angularVelocity, state.desiredAttitude.
     * Writes state.finDeflections (clamped to ±maxDeflection degrees).
     */
    void update(RocketState& state, double dt, double maxDeflectionDeg);

    /** For logging: last commanded rates and control outputs. */
    double pitchRate = 0, yawRate = 0, rollRate = 0;
    double pitchCtrl = 0, yawCtrl = 0, rollCtrl = 0;

private:
    double timeSinceOuter_ = 0.0;
    double timeSinceInner_ = 0.0;
};
