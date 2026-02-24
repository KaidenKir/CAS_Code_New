#include <iostream>
#include "RocketConfig.h"
#include "Motor.h"
#include "RocketSimulation.h"

using namespace std;

int main() {
    // ── Load motor file (.rse format from RASAero / OpenRocket) ──────────
    Motor motor;
    if (!motor.loadFromFile("AeroTech_M1350W.rse")) {
        cerr << "Failed to load motor file." << endl;
        return 1;
    }

    // ── Load rocket simulation data (CSV from OpenRocket export) ─────────
    RocketConfig config;
    if (!config.loadSimData("Kaidens_L3_Data.csv")) {
        cerr << "Failed to load rocket sim data." << endl;
        return 1;
    }

    // ── Create simulation ─────────────────────────────────────────────────
    RocketSimulation sim(config, motor);

    // ── Tune PID gains ────────────────────────────────────────────────────
    // Outer loop: converts attitude error (rad) → rate command (rad/s)
    // Inner loop: converts rate error (rad/s)  → fin deflection (rad, then deg)
    // All zeros = open-loop (no active stabilisation).  Tune to your rocket!
    sim.cas().setOuterGains(/*Kp*/ 0.0, /*Ki*/ 0.0, /*Kd*/ 0.0);
    sim.cas().setInnerGains(/*Kp*/ 0.0, /*Ki*/ 0.0, /*Kd*/ 0.0);

    // ── Set initial conditions ────────────────────────────────────────────
    double launchAngleDeg = 15.0;
    Vector3d perturbation(0.0, 0.0, 0.0);  // roll, pitch, yaw offset (rad)
    Vector3d omega       (0.0, 0.0, 0.0);  // initial angular rates   (rad/s)

    sim.setInitialConditions(launchAngleDeg, perturbation, omega);

    // ── Run ───────────────────────────────────────────────────────────────
    if (sim.run(30.0, "rocket_flight_data.csv")) {
        cout << "Done. Data saved to rocket_flight_data.csv" << endl;
    } else {
        cerr << "Simulation failed." << endl;
        return 1;
    }

    return 0;
}
