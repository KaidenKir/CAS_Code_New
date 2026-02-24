#include "PIDController.h"
#include <algorithm>
#include <cmath>

using namespace std;

// ═════════════════════════════════════════════════════════════════════════════
//  PIDController
// ═════════════════════════════════════════════════════════════════════════════

void PIDController::setGains(double kp, double ki, double kd) {
    Kp = kp; Ki = ki; Kd = kd;
}

void PIDController::reset() {
    integral_    = 0.0;
    prevActual_  = 0.0;
    dFiltered_   = 0.0;
    alpha_       = 0.0;
    initialized_ = false;
}

/**
 * One-step PID update:
 *
 *  error  = goal - actual
 *
 *  P term = Kp * error
 *
 *  I term = Ki * ∫error dt
 *    Anti-windup: if the integrator has wound up in the opposite direction
 *    to the current error, clear it — this prevents the integrator from
 *    fighting the controller when the setpoint changes sign.
 *
 *  D term = Kd * d(error)/dt  (low-pass filtered to reduce noise)
 *    We differentiate -actual instead of error so a step change in the
 *    setpoint doesn't produce a large derivative spike ("derivative kick").
 *    Filter: dFiltered = (1-alpha)*dFiltered + alpha * raw_d
 *    alpha = dt / (tau + dt)   — larger tau → smoother, more lag
 */
double PIDController::update(double actual, double goal, double dt) {
    if (!initialized_) {
        prevActual_  = actual;
        dFiltered_   = 0.0;
        initialized_ = true;
        alpha_       = dt / (tau + dt);
    }

    double error = goal - actual;

    // Anti-windup: clear integral if it's pushing the wrong way
    if (sign(error) != sign(integral_)) integral_ = 0.0;

    integral_ += error * dt;
    integral_  = clamp(integral_, -integralLimit, integralLimit);

    // Derivative on measurement (avoids setpoint kick)
    double raw_d  = -(actual - prevActual_) / dt;
    dFiltered_    = (1.0 - alpha_) * dFiltered_ + alpha_ * raw_d;

    prevActual_ = actual;

    double output = Kp * error + Ki * integral_ + Kd * dFiltered_;
    return clamp(output, -outputLimit, outputLimit);
}

// ═════════════════════════════════════════════════════════════════════════════
//  CASController
// ═════════════════════════════════════════════════════════════════════════════

void CASController::setOuterGains(double kp, double ki, double kd) {
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
 * Two-loop (cascade) update.
 *
 * OUTER LOOP — runs every outerDt seconds:
 *   1. Compute attitude error quaternion:
 *        q_err = q_current_inverse * q_desired
 *      If q_current were equal to q_desired this would give the identity
 *      quaternion (no error).
 *   2. Convert q_err to an axis-angle vector.  The three components of that
 *      vector are the pitch, yaw, and roll errors in radians.
 *   3. Feed each error into its outer PID controller to get a rate command
 *      (rad/s) for the inner loop.
 *
 * INNER LOOP — runs every innerDt seconds:
 *   1. Measure the body angular rates from state.angularVelocity.
 *   2. Feed each (measured rate, commanded rate) pair into its inner PID to
 *      get a fin-deflection command in radians.
 *   3. Map pitch/yaw/roll commands onto the four fins and convert to degrees.
 *
 * Fin mixing (standard X-configuration):
 *   fin[0] (+Y fin): +pitch, +roll
 *   fin[1] (+X fin): +yaw,   +roll
 *   fin[2] (-Y fin): -pitch, +roll
 *   fin[3] (-X fin): -yaw,   +roll
 */
void CASController::update(RocketState& state, double dt, double maxDeflectionDeg) {
    timeSinceOuter_ += dt;
    timeSinceInner_ += dt;

    // ── Outer loop ────────────────────────────────────────────────────────
    if (timeSinceOuter_ >= outerDt) {
        // Attitude error in body frame
        Quaterniond qErr = state.attitude.conjugate() * state.desiredAttitude;
        Vector3d    errVec = quaternionToAngles(qErr);   // [pitch_err, yaw_err, roll_err]

        // Each component drives toward zero → output is the desired rate
        pitchRate = pitchOuter.update(errVec[1], 0.0, outerDt);
        yawRate   = yawOuter  .update(errVec[0], 0.0, outerDt);
        rollRate  = rollOuter .update(errVec[2], 0.0, outerDt);

        timeSinceOuter_ = 0.0;
    }

    // ── Inner loop ────────────────────────────────────────────────────────
    if (timeSinceInner_ >= innerDt) {
        // body-frame rates: angularVelocity = [roll, pitch, yaw]
        pitchCtrl = pitchInner.update(state.angularVelocity[1], pitchRate, innerDt);
        yawCtrl   = yawInner  .update(state.angularVelocity[2], yawRate,   innerDt);
        rollCtrl  = rollInner .update(state.angularVelocity[0], rollRate,  innerDt);

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
