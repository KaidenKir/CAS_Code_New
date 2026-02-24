#pragma once 

#include <vector>
#include "MathUtils.h"
#include "RocketState.h" //Need to make this file

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
 * For each instance of the controller that is called, outputLimit must be defined. It is initalized as 0.0
 *
 * Example:
 *   PIDController pitchRate;
 *   pitchRate.setGains(2.0, 0.1, 0.5);    // Kp, Ki, Kd
 *   double cmd = pitchRate.update(measured, desired, 0.002);
 */
class PIDController{
public:
    // Tuning Inputs --------------------
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    double outputLimit   = 0.0;  // |output| is clamped to this
    double integralLimit = 10.0; // |integral| is clamped to this
    double tau           = 0.02; // derivative filter time constant (s)
     
    void setGains(double kp, double ki, double kd);

    /**
     * Advances the controller by one time-step.
     * 
     * 'actual' - the measured value of state
     * 'goal'   - the desired value
     * 'dt'     - elapsed time since the last call (s)
     * 
     * Retuerns controller output
     */
    double update( double actual, double goal, double dt);

    /** Clears internal state */
    void reset();

private:
    double integral_   = 0.0;
    double prevActual_ = 0.0;
    double dFiltered_  = 0.0;
    double alpha_      = 0.0;
    bool initialized_    = false;
}

class CASController{
public:
    double outerDt = 0.02;  // outer loop update period
    double innerDt = 0.002; // inner loop update period

    // outer loop controllers
    PIDController pitchOuter, yawOuter, rollOuter;

    // inner loop controllers
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
}