#include "RocketSimulation.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace std;

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────

RocketSimulation::RocketSimulation(const RocketConfig& config, const Motor& motor)
    : config_(config), motor_(motor)
{
    double motorDry     = motor_.initialWeight - motor_.propellantWeight;
    state_.mass = config_.massDry + motorDry + motor_.propellantWeight;

    // ── Default PID gains (all zero = open-loop, no correction) ───────────
    // Tune these before calling run():
    //   sim.cas().setOuterGains(Kp, Ki, Kd);
    //   sim.cas().setInnerGains(Kp, Ki, Kd);
    cas_.setOuterGains(0.0, 0.0, 0.0);
    cas_.setInnerGains(0.0, 0.0, 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Initial conditions
// ─────────────────────────────────────────────────────────────────────────────

void RocketSimulation::setInitialConditions(double launchAngleDeg,
                                            Vector3d perturbation,
                                            Vector3d omega) {
    state_.reset();

    double launchAngleRad = launchAngleDeg * M_PI / 180.0;
    state_.attitude = eulerToQuaternion(Vector3d(
        perturbation[0],
        launchAngleRad + perturbation[1],
        perturbation[2]
    ));
    state_.angularVelocity = omega;

    double motorDry = motor_.initialWeight - motor_.propellantWeight;
    state_.mass = config_.massDry + motorDry + motor_.propellantWeight;

    casOnline_ = false;
    logger_.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Aerodynamics
// ─────────────────────────────────────────────────────────────────────────────

pair<Vector3d, Vector3d>
RocketSimulation::aeroForcesAndMoments(const RocketState& s) const {
    // Guard against bad velocity input
    if (s.velocity.hasNaN()) {
        cout << "Aero: velocity NaN at t=" << s.time << endl;
        return {Vector3d::Zero(), Vector3d::Zero()};
    }

    Atmosphere atm = Atmosphere::at(s.position[2]);
    if (!std::isfinite(atm.density)) {
        cout << "Aero: bad atmosphere at alt=" << s.position[2] << endl;
        return {Vector3d::Zero(), Vector3d::Zero()};
    }

    double speed = s.velocity.norm();
    if (speed < 0.1)                          // avoid ÷0 at rest
        return {Vector3d::Zero(), Vector3d::Zero()};

    Vector3d force  = Vector3d::Zero();
    Vector3d moment = Vector3d::Zero();

    Vector3d velUnit = s.velocity / speed;
    double   dynQ   = 0.5 * atm.density * speed * speed;

    // Velocity in the body frame (needed for canards)
    Vector3d velBody = s.attitude.conjugate()._transformVector(s.velocity);

    // ── Axial drag ────────────────────────────────────────────────────────
    // Drag opposes the velocity vector; magnitude = q * Cd * A_ref
    force += -dynQ * config_.cd * config_.referenceArea * velUnit;

    // ── Body normal force (angle of attack) ───────────────────────────────
    Vector3d rocketAxis = s.attitude._transformVector(Vector3d::UnitZ());
    double cosAlpha = clamp(velUnit.dot(rocketAxis), -1.0, 1.0);
    double alpha    = acos(cosAlpha);

    auto [cg, cp] = config_.getCGandCP(s.mass, speed);
    double stabilityArm = -(cg - cp);   // positive = stable (CP behind CG)

    if (fabs(alpha) > 1e-6) {
        Vector3d normalDir = rocketAxis.cross(velUnit);
        double normMag = normalDir.norm();
        if (normMag > 1e-6) {
            normalDir /= normMag;
            double Cn = config_.Cn_alpha * sin(alpha);
            double Fn = dynQ * config_.referenceArea * Cn;
            Vector3d normalForce = Fn * normalDir;
            force  += normalForce;
            // Stability moment: normal force acts at CP, moment arm from CG
            moment += Vector3d(0, 0, stabilityArm).cross(normalForce);
        }
    }

    // ── Canard forces and moments ─────────────────────────────────────────
    if (speed > 1e-6) {
        double canardChord     = config_.finArea / config_.span;
        double canardAeroCenter = -(config_.finLocation + 0.25 * canardChord) - cg;
        double canardAeroSpan   = config_.diameter * 0.5 + config_.span * 0.5;

        // Position of each fin's aerodynamic center in the body frame
        // Order: +Y, -Y, +X, -X
        const Vector3d finPositions[4] = {
            { 0,  canardAeroSpan, canardAeroCenter},
            { 0, -canardAeroSpan, canardAeroCenter},
            { canardAeroSpan, 0, canardAeroCenter},
            {-canardAeroSpan, 0, canardAeroCenter},
        };
        // Span unit vectors for each fin
        const Vector3d finSpans[4] = {
            {0,  1, 0},
            {0, -1, 0},
            {1,  0, 0},
            {-1, 0, 0},
        };

        for (int i = 0; i < 4; i++) {
            double deflRad   = s.finDeflections[i] * M_PI / 180.0;
            Vector3d finPos  = finPositions[i];
            Vector3d spanVec = finSpans[i];

            // Local velocity at the fin = body velocity + omega × r
            Vector3d localVel = velBody + s.angularVelocity.cross(finPos);
            double localSpeed = localVel.norm();
            if (localSpeed < 1e-6) continue;

            double localDynQ = 0.5 * atm.density * localSpeed * localSpeed;

            // Lift direction = v × span_vector  (normalised)
            Vector3d liftDir = localVel.cross(spanVec);
            if (liftDir.norm() < 1e-8) continue;
            liftDir.normalize();

            // Remove the span-wise component to get velocity in the fin plane
            Vector3d velInPlane = localVel - localVel.dot(spanVec) * spanVec;
            if (!velInPlane.allFinite()) continue;

            // Geometric AoA: angle between the axial direction and the
            // velocity component in the fin's chord plane.
            Vector3d perpDir = Vector3d::UnitZ().cross(spanVec);
            if (perpDir.norm() < 1e-8) perpDir = Vector3d::UnitX().cross(spanVec);
            perpDir.normalize();

            double axialComp = velInPlane[2];
            double perpComp  = velInPlane.dot(perpDir);
            double geoAoA    = atan2(perpComp, axialComp);
            if (!std::isfinite(geoAoA)) continue;

            double effAoA = geoAoA + deflRad;

            // Flip lift direction so positive deflection → positive lift
            if (effAoA * liftDir.dot(perpDir) < 0) liftDir = -liftDir;

            // Lift-curve slope via Prandtl-Glauert / supersonic corrections
            double AR = config_.finAspectRatio;
            double M  = localSpeed / atm.speedOfSound;
            double Cl_alpha;
            if (M < 0.8) {
                double beta = sqrt(1.0 - M*M);
                Cl_alpha = (2*M_PI*AR) / (2 + sqrt(4 + AR*AR*(1-M*M)/(beta*beta)));
            } else if (M < 1.2) {
                // Transonic: fix at M=0.9 to avoid singularity
                double M_est = 0.9, beta = sqrt(1 - M_est*M_est);
                Cl_alpha = (2*M_PI*AR) / (2 + sqrt(4 + AR*AR*(1-M_est*M_est)/(beta*beta)));
            } else {
                Cl_alpha = (4*AR) / sqrt(M*M - 1);
            }
            if (!std::isfinite(Cl_alpha)) continue;

            double Cl = Cl_alpha * effAoA;
            if (!std::isfinite(Cl)) continue;

            // Lift force (body frame)
            Vector3d liftBody = localDynQ * config_.finArea * Cl * liftDir;
            if (!liftBody.allFinite()) continue;

            // Drag (induced + wave + form)
            double finEff    = 0.85;
            double Cd_ind    = (AR > 0) ? Cl*Cl / (M_PI * AR * finEff) : 0.0;
            double Cd_wave   = (M >= 0.8) ? 0.5*(M-0.8)*(M-0.8) : 0.0;
            double Cd_form   = 0.01;
            double Cd_fin    = Cd_ind + Cd_wave + Cd_form;
            Vector3d dragBody = localDynQ * config_.finArea * Cd_fin * (-localVel.normalized());

            Vector3d totalFinForce  = liftBody + dragBody;
            Vector3d totalFinMoment = finPos.cross(totalFinForce);

            moment += totalFinMoment;
            // Fin forces are computed in the body frame; transform to world
            force  += s.attitude._transformVector(totalFinForce);
        }
    }

    return {force, moment};
}

// ─────────────────────────────────────────────────────────────────────────────
//  Equations of motion
// ─────────────────────────────────────────────────────────────────────────────

StateDerivative RocketSimulation::derivatives(const RocketState& s,
                                              double time,
                                              double thrustForce) const {
    StateDerivative d;

    // Recompute mass from the propellant table (important for mid-step RK4 calls)
    double propMass = motor_.propMassAt(time);
    double motorDry = motor_.initialWeight - motor_.propellantWeight;
    double mass     = config_.massDry + motorDry + propMass;

    auto [aeroForce, aeroMoment] = aeroForcesAndMoments(s);

    // Thrust acts along body +Z, rotated to world frame
    Vector3d thrustWorld = s.attitude._transformVector(Vector3d(0, 0, thrustForce));

    // Gravity acts in world −Z
    Vector3d gravity(0, 0, -9.81 * mass);

    // ── Translational EOM ─────────────────────────────────────────────────
    d.velocity     = s.velocity;                        // dx/dt = v
    d.acceleration = (thrustWorld + aeroForce + gravity) / mass;  // dv/dt = F/m

    // ── Rotational EOM (Euler's equations in the body frame) ──────────────
    auto [lMOI, rMOI] = config_.getMOI(mass);
    Matrix3d I = Matrix3d::Zero();
    I(0,0) = rMOI;  I(1,1) = rMOI;  I(2,2) = lMOI;

    Vector3d omega = s.angularVelocity;

    // d(ω)/dt = I⁻¹ · (M - ω × (I·ω))
    // The term  ω × (I·ω)  is the gyroscopic (Coriolis) effect: when the
    // rocket spins, the angular momentum vector tries to stay fixed in space,
    // creating an apparent torque that needs to be subtracted.
    Vector3d gyroscopic   = omega.cross(I * omega);
    d.angularAcceleration = I.inverse() * (aeroMoment - gyroscopic);

    // ── Quaternion derivative ─────────────────────────────────────────────
    // d(q)/dt = 0.5 * q * ω_quat
    // where ω_quat is the angular velocity as a pure quaternion (0, ωx, ωy, ωz).
    // This propagates the attitude quaternion forward in time.
    Quaterniond q       = s.attitude.normalized();
    Quaterniond omegaQ(0, omega.x(), omega.y(), omega.z());
    d.quaternionDerivative = scaleQuaternion(q * omegaQ, 0.5);

    return d;
}

// ─────────────────────────────────────────────────────────────────────────────
//  RK4 helpers
// ─────────────────────────────────────────────────────────────────────────────

RocketState RocketSimulation::applyDerivative(const RocketState& s,
                                              const StateDerivative& d,
                                              double scale) const {
    RocketState ns = s;
    ns.position        += d.velocity            * scale;
    ns.velocity        += d.acceleration        * scale;
    ns.angularVelocity += d.angularAcceleration * scale;

    // Add the scaled quaternion increment component-wise (not normalised yet)
    Quaterniond dq = scaleQuaternion(d.quaternionDerivative, scale);
    ns.attitude = Quaterniond(s.attitude.w() + dq.w(),
                              s.attitude.x() + dq.x(),
                              s.attitude.y() + dq.y(),
                              s.attitude.z() + dq.z());
    ns.attitude.normalize();
    return ns;
}

/**
 * 4th-order Runge-Kutta integration.
 *
 * RK4 approximates the solution to dy/dt = f(t,y) as:
 *
 *   k1 = f(t,        y)
 *   k2 = f(t + dt/2, y + k1*dt/2)
 *   k3 = f(t + dt/2, y + k2*dt/2)
 *   k4 = f(t + dt,   y + k3*dt  )
 *
 *   y_new = y + (dt/6) * (k1 + 2k2 + 2k3 + k4)
 *
 * Using four evaluations of the derivative instead of one (Euler) makes
 * the method 4th-order accurate — errors are proportional to dt⁴ instead
 * of dt, allowing much larger timesteps for the same accuracy.
 */
void RocketSimulation::integrateRK4(double thrustForce) {
    // Refresh mass from motor table
    double propMass = motor_.propMassAt(state_.time);
    double motorDry = motor_.initialWeight - motor_.propellantWeight;
    state_.mass     = config_.massDry + motorDry + propMass;

    // k1 at current state
    StateDerivative k1 = derivatives(state_, state_.time, thrustForce);

    // k2 at half-step using k1
    RocketState s2 = applyDerivative(state_, k1, dt_ / 2.0);
    s2.time        = state_.time + dt_ / 2.0;
    StateDerivative k2 = derivatives(s2, s2.time, thrustForce);

    // k3 at half-step using k2
    RocketState s3 = applyDerivative(state_, k2, dt_ / 2.0);
    s3.time        = state_.time + dt_ / 2.0;
    StateDerivative k3 = derivatives(s3, s3.time, thrustForce);

    // k4 at full step using k3
    RocketState s4 = applyDerivative(state_, k3, dt_);
    s4.time        = state_.time + dt_;
    StateDerivative k4 = derivatives(s4, s4.time, thrustForce);

    // Weighted sum
    double w = dt_ / 6.0;
    state_.position        += w * (k1.velocity            + 2*k2.velocity            + 2*k3.velocity            + k4.velocity           );
    state_.velocity        += w * (k1.acceleration        + 2*k2.acceleration        + 2*k3.acceleration        + k4.acceleration       );
    state_.angularVelocity += w * (k1.angularAcceleration + 2*k2.angularAcceleration + 2*k3.angularAcceleration + k4.angularAcceleration );

    // Quaternion weighted sum (component-wise, then normalise)
    auto wQ = [&](const StateDerivative& k) { return scaleQuaternion(k.quaternionDerivative, 1.0); };
    Quaterniond dq(
        w * (k1.quaternionDerivative.w() + 2*k2.quaternionDerivative.w() + 2*k3.quaternionDerivative.w() + k4.quaternionDerivative.w()),
        w * (k1.quaternionDerivative.x() + 2*k2.quaternionDerivative.x() + 2*k3.quaternionDerivative.x() + k4.quaternionDerivative.x()),
        w * (k1.quaternionDerivative.y() + 2*k2.quaternionDerivative.y() + 2*k3.quaternionDerivative.y() + k4.quaternionDerivative.y()),
        w * (k1.quaternionDerivative.z() + 2*k2.quaternionDerivative.z() + 2*k3.quaternionDerivative.z() + k4.quaternionDerivative.z())
    );
    state_.attitude = Quaterniond(state_.attitude.w() + dq.w(),
                                  state_.attitude.x() + dq.x(),
                                  state_.attitude.y() + dq.y(),
                                  state_.attitude.z() + dq.z());
    state_.attitude.normalize();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Data logging
// ─────────────────────────────────────────────────────────────────────────────

void RocketSimulation::logCurrentState() {
    Atmosphere atm   = Atmosphere::at(state_.position[2]);
    double speed     = state_.velocity.norm();
    double mach      = speed / atm.speedOfSound;
    double dynQ      = 0.5 * atm.density * speed * speed;
    double thrust    = motor_.thrustAt(state_.time);

    auto [aeroForce, aeroMoment] = aeroForcesAndMoments(state_);

    // Euler angles from rotation matrix (ZYX convention → yaw, pitch, roll)
    Vector3d euler = state_.attitude.toRotationMatrix().eulerAngles(2, 1, 0);

    FlightLogger::Sample s;
    s.time               = state_.time;
    s.positionX          = state_.position[0];
    s.positionY          = state_.position[1];
    s.positionZ          = state_.position[2];
    s.velocityMag        = speed;
    s.verticalVelocity   = state_.velocity[2];
    s.horizontalVelocity = sqrt(state_.velocity[0]*state_.velocity[0] +
                                state_.velocity[1]*state_.velocity[1]);
    s.accelerationMag    = (aeroForce.norm() + thrust) / state_.mass;
    s.roll               = euler[2] * 180.0 / M_PI;
    s.pitch              = euler[1] * 180.0 / M_PI;
    s.yaw                = euler[0] * 180.0 / M_PI;
    s.rollRate           = state_.angularVelocity[0] * 180.0 / M_PI;
    s.pitchRate          = state_.angularVelocity[1] * 180.0 / M_PI;
    s.yawRate            = state_.angularVelocity[2] * 180.0 / M_PI;
    s.thrust             = thrust;
    s.drag               = aeroForce.norm();
    s.mass               = state_.mass;
    s.reqRollRate        = cas_.rollRate;
    s.reqPitchRate       = cas_.pitchRate;
    s.reqYawRate         = cas_.yawRate;
    s.reqRollCtrl        = cas_.rollCtrl;
    s.reqPitchCtrl       = cas_.pitchCtrl;
    s.reqYawCtrl         = cas_.yawCtrl;
    s.fin1               = state_.finDeflections[0];
    s.fin2               = state_.finDeflections[1];
    s.fin3               = state_.finDeflections[2];
    s.fin4               = state_.finDeflections[3];
    s.machNumber         = mach;
    s.dynamicPressure    = dynQ;

    logger_.record(s);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main simulation loop
// ─────────────────────────────────────────────────────────────────────────────

bool RocketSimulation::run(double maxTime, const string& outputFile, int logInterval) {
    cout << "Starting simulation  |  initial mass: " << state_.mass << " kg"
         << "  |  max time: " << maxTime << " s" << endl;

    auto wallStart = chrono::high_resolution_clock::now();
    auto lastPrint = wallStart;

    // Stop if rocket hits ground (small negative tolerance for launch pad)
    while (state_.time < maxTime && state_.position[2] > -10.0) {
        double thrust = motor_.thrustAt(state_.time);
        double speed  = state_.velocity.norm();

        // ── CAS on/off logic ──────────────────────────────────────────────
        // The CAS only operates above 100 m/s; below that there isn't enough
        // dynamic pressure for the fins to be effective.
        if (casOnline_ && speed < 100.0) {
            casOnline_ = false;
            cout << "CAS offline at t=" << state_.time << " s (speed < 100 m/s)" << endl;
        } else if (!casOnline_ && speed >= 100.0 && thrust <= 0.0) {
            casOnline_ = true;
            cout << "CAS online  at t=" << state_.time << " s" << endl;
        }

        if (casOnline_) {
            cas_.update(state_, dt_, config_.maxFinDeflection);
        } else {
            // Zero fins when CAS is off
            state_.finDeflections = Vector4d::Zero();
        }

        // ── Integrate ─────────────────────────────────────────────────────
        integrateRK4(thrust);

        state_.time += dt_;
        state_.step++;

        // ── Log periodically ──────────────────────────────────────────────
        if (state_.step % logInterval == 0)
            logCurrentState();

        // ── Progress print ────────────────────────────────────────────────
        auto now     = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::seconds>(now - lastPrint);
        if (elapsed.count() >= 5 || state_.step % 1000 == 0) {
            cout << "t=" << fixed << setprecision(2) << state_.time
                 << " s  alt=" << state_.position[2]
                 << " m  vel=" << speed << " m/s" << endl;
            lastPrint = now;
        }
    }

    // Always capture the final state
    logCurrentState();

    cout << "Simulation complete  |  alt=" << state_.position[2]
         << " m  |  t=" << state_.time << " s" << endl;

    return logger_.saveCSV(outputFile);
}
