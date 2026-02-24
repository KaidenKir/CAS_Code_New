//TODO: Need to finish a bunch of the files, Im switching to finish the .h files that are needed
#pragma once

#include <string>
#include "rocketConfig.h"       // TO-DO: NEED TO MAKE THIS FILE
#include "motor.h"
#include "rocketState.h"
#include "PIDController.h"
#include "flightLogger.h"       // TO-DO: NEED TO MAKE THIS FILE
#include "atmosphere.h"         // TO-DO: NEED TO MAKE THIS FILE
#include "mathUtils.h"

/**
 * RocketSimulation
 * ----------------
 * Top-level simulation class that ties together all the subsystems:
 *   - Equations of motion (6-DOF rigid body)
 *   - Aerodynamics (drag, body lift, canard lift)
 *   - Motor thrust curve interpolation
 *   - CAS (Control Augmentation System) via CASController
 *   - RK4 numerical integration
 *   - Flight data logging
 *
 * Typical usage:
 *
 *   RocketSimulation sim(config, motor);
 *   sim.cas().setOuterGains(1.0, 0.05, 0.2);   // tune outer PID
 *   sim.cas().setInnerGains(3.0, 0.1,  0.5);   // tune inner PID
 *   sim.setInitialConditions(launchAngleDeg, perturbation, omega);
 *   sim.run(30.0, "output.csv");
 */
class RocketSimulation{
public:
    outerKp = 0.0;
    outerKi = 0.0;
    outerKd = 0.0;

    innerKp = 0.0;
    innerKi = 0.0;
    innerKd = 0.0;

    rocketSimulation(const RocketConfig& config, const Motor& motor);

    //settup

    /**
     * Sets initial state before calling run().
     *
     * launchAngleDeg   – tilt from vertical in the pitch plane (degrees).
     * perturbation     – additional (roll, pitch, yaw) Euler offset (radians).
     * omega            – initial body angular velocity (rad/s).
     */
    void setInitialConditions(double launchAngleDeg                  = 0.0,
                              Vector3d perturbation = Vector3d::Zero(),
                              Vector3d omega        = Vector3d::Zero());

    CASController& cas() {return cas_;}

    //simulation

    /**
     * Runs the simulation until maxTime (seconds) or ground impact.
     * Logs data every logInterval timesteps and saves results to outputFile.
     * Returns true on success.
     */
    bool run(double maxTime, const std::string& outputFile, int logInterval = 10);

private:
    //subsystems
    RocketConfig  config_;
    Motor         motor_;
    RocketState   state_;
    CASController cas_;
    FlightLogger  logger_;

    //simulation parameters
    double dt_ = 0.001; // integration timestep (1ms)
    bool casOnline_ = false;

    //physics
    /**
     * Returns {total aero force (world frame), total aero moment (body frame)}
     * given the current rocket state.
     *
     * Includes:
     *  - Axial drag
     *  - Body normal force (from angle of attack)
     *  - Stability moment (body normal force acting at CP)
     *  - Canard lift, drag, and moment (for all four fins)
     */
    std::pair<Vector3d, Vector3d> aeroForcesAndMoments(const RocketState& s) const;

    /**
     * Computes the time derivatives of the state vector.
     * Called four times per RK4 step.
     *
     * thrustForce – current motor thrust in Newtons (held constant during RK4).
     */
    StateDerivative derivatives(const RocketState& s, double time, double thrustForce) const;

    /**
     * Returns a new state equal to  s + deriv * scale.
     * Used internally by the RK4 integrator to build intermediate states.
     */
    RocketState applyDerivative(const RocketState& s,
                                const StateDerivative& d,
                                double scale) const;

    /** Advances state_ by one timestep using 4th-order Runge-Kutta. */
    void integrateRK4(double thrustForce);

    // ── Logging ───────────────────────────────────────────────────────────

    /** Captures the current state into the FlightLogger. */
    void logCurrentState();
}