#pragma once
#include <string>
#include <utility>
#include "Eigen/Eigen/Dense"

using namespace Eigen;

/**
 * RocketConfig
 * ------------
 * Stores all fixed physical properties of the rocket airframe and fins,
 * plus the variable data table (velocity, mass, MOI, CG, CP) that was
 * pre-computed by an external sim (e.g. OpenRocket) and saved to a CSV.
 *
 * After calling loadSimData() the following interpolation queries are
 * available:
 *   getMOI(mass)           → {longitudinal MOI, radial MOI}  kg·m²
 *   getCGandCP(mass, vel)  → {CG position, CP position}      metres
 */
class RocketConfig {
public:
    // ── Airframe ──────────────────────────────────────────────────────────
    double massDry       = 4.57;     // kg   — structural + avionics mass (no motor)
    double length        = 2.0;      // m    — rocket length
    double diameter      = 0.104;    // m    — outer diameter
    double Cn_alpha      = 11.065;   // 1/rad — normal-force slope (body + fins)

    // ── Aerodynamics ──────────────────────────────────────────────────────
    double cd            = 0.406;    // —     — drag coefficient
    double referenceArea = 0.00849;  // m²   — cross-sectional reference area

    // ── Control fins (four identical fins assumed) ─────────────────────────
    double span             = 0.038;   // m    — fin semi-span
    double rootChord        = 0.089;   // m
    double tipChord         = 0.025;   // m
    double finArea          = 0.0;     // m²   — calculated in constructor
    double finAspectRatio   = 0.0;     // —    — calculated in constructor
    double finLocation      = 0.3;     // m    — distance of fin AC from nose tip
    double maxFinDeflection = 10.0;    // deg  — hard limit on servo travel

    /** Computes derived fin geometry and loads sim data from a CSV file.
     *  Returns false on failure. */
    bool loadSimData(const std::string& filename);

    /** Returns {longitudinal MOI, radial MOI} (kg·m²) at the given total mass. */
    std::pair<double, double> getMOI(double mass) const;

    /** Returns {CG position, CP position} (m, measured from nose, negative forward)
     *  as a function of total mass and airspeed. */
    std::pair<double, double> getCGandCP(double mass, double velocity) const;

private:
    /**
     * Each row of simData holds:
     *   col 0 – velocity (m/s)
     *   col 1 – total mass (kg)
     *   col 2 – longitudinal MOI (kg·m²)
     *   col 3 – radial MOI (kg·m²)
     *   col 4 – CG position (m from nose)
     *   col 5 – CP position (m from nose)
     */
    Matrix<double, Dynamic, 6> simData_;

    // Scalar linear interpolation helper
    static double lerp(double alpha, double a, double b) { return a + alpha * (b - a); }
};
