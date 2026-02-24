#include "PIDController.h"
#include <algorithm>
#include <cmath>

using namespace std;

//----------------------------------------------------------------------------------------------------
// PIDController
// ---------------------------------------------------------------------------------------------------

void PIDController::setGains(double kp, double ki, double kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::reset() {
    integral_    = 0.0;
    prevActual_  = 0.0;
    dFiltered_   = 0.0;
    alpha_       = 0.0;
    initialized_ = false;
}

/**
 * One step PID update:
 * 
 * error = goal - actual
 * 
 * P term = Kp * error
 * 
 * I term = Ki * (error * dt + I term)
 *     I term has anti windup, so if the I term swithes signs, 
 *      it will reset to 0.0 to prevent it fighting in the wrong 
 *      direction
 * 
 * D term = Kd * d(error)/dt
 *     We differentiate -actual instead of error so a step change in the
 *      setpoint doesn't produce a large derivative spike ("derivative kick").
 *      Filter: dFiltered = (1-alpha)*dFiltered + alpha * raw_d
 *      alpha = dt / (tau + dt)   — larger tau → smoother, more lag
 */
double PIDController::update(double actual, double goal, double dt){
    if (!initialized_) {
        prevActual_ = actual;
        dFiltered_ = 0.0;
        initialized_ = true;
        alpha_ = dt/(tau + dt);
    }

    double error = (goal - actual);

    if(sign(error) != sign(integral_)) integral_ = 0.0;

    integral_ += error * dt;
    integral_ = clamp(integral_, -integralLimit, integralLimit);

    double de_dt = -(actual - prevActual_)/dt;
    dFiltered_ = (1.0 - alpha_) * dFiltered_ + alpha_ * de_dt;

    double pTerm = error * Kp;
    double iTerm = integral_ * Ki;
    double dTerm = dFiltered_ * Kd;

    prevActual_ = actual;

    return clamp(pTerm + iTerm + dTerm, -outputLimit, outputLimit);
}

// ---------------------------------------------------------------------------------------------------
// CASController
// ---------------------------------------------------------------------------------------------------

void CASController::setOuterGains(double kp, double ki, double kd){
    pitchOuter.setGains(kp, ki, kd);
    yawOuter  .setGains(kp, ki, kd);
    rollOuter .setGains(kp, ki, kd);
}

void CASController::setInnerGains(double kp, double ki, double kd) {
    pitchInner.setGains(kp, ki, kd);
    yawInner  .setGains(kp, ki, kd);
    rollInner .setGains(kp, ki, kd);
}

/**
 * Two loop cascaded update
 * 
 * OUTER LOOP: Runs every 'outerDt' seconds:
 *  1. Compute attitude error quaternion:
 *      q_err = q_current_inverse * q_desired
 *     If q_quaternion == q_desired, identity quaternion is returned
 *  2. Convert q_err go a axis-angle vector. The three compondents of this are pitch, yaw , and roll errors\
 *  3. Feed each error into its outer PID controller to get desired rad/s for inner loop
 * 
 * INNER LOOP: runs every 'innerDt' seconds:
 *  1. Measure the body angular rates from state.angularVelocity.
 *  2. Feed each (measured rate, commanded rate) pair into its inner PID to
 *      get a fin-deflection command in radians.
 *  3. Map pitch/yaw/roll commands onto the four fins and convert to degrees.
 * 
 * Fin mixing (standard X-configuration):
 *   fin[0] (+Y fin): +pitch, +roll
 *   fin[1] (+X fin): +yaw,   +roll
 *   fin[2] (-Y fin): -pitch, +roll
 *   fin[3] (-X fin): -yaw,   +roll
 */
void CASController::update(RocketState& state, double dt, double maxDeflectionDeg){
    timeSinceOuter_ += dt;
    timeSinceInner_ += dt;

    // outer loop commands
    if(timeSinceOuter_ >= outerDt){
        Quaterniond qErr = state.attitude.conjugate() * state.desiredAttitude;
        Vector3d errVec = quaternionToAngles(qErr);
        //cout << "yaw error: " << errVec[0] << " | pitch error: " << errVec[1] << " | roll error: " << errVec[2] << endl;

        yawRate = yawOuter.update(errVec[0], 0.0, timeSinceOuter_);
        pitchRate = pitchOuter.update(errVec[1], 0.0, timeSinceOuter_);
        rollRate = rollOuter.update(errVec[2], 0.0, timeSinceOuter_);

        timeSinceOuter_ = 0.0;
    }

    // inner loop  commands
    if(timeSinceInner_ >= innerDt){
        yawCtrl = yawInner.update(state.angularVelocity[2], yawRate, timeSinceInner_);
        pitchCtrl = pitchInner.update(errVec[1], pitchRate, timeSinceInner_);
        rollCtrl = rollInner.update(errVec[0], rollRate, timeSinceInner_);

        // Fin mixing: convert rad → deg and clamp
        auto finDeg = [&](double cmd) {
            return clamp(cmd * 180.0 / M_PI, -maxDeflectionDeg, maxDeflectionDeg);
        };
        state.finDeflections[0] = finDeg( pitchCtrl + rollCtrl);   // +Y
        state.finDeflections[1] = finDeg( yawCtrl   + rollCtrl);   // +X
        state.finDeflections[2] = finDeg(-pitchCtrl + rollCtrl);   // -Y
        state.finDeflections[3] = finDeg(-yawCtrl   + rollCtrl);   // -X

        timeSinceInner_ = 0.0;
    }
}