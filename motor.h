#pragma once

#include <string>
#include "Eigen/Eigen/Dense"

using namespace Eigen;

/**
 * Motor
 * -----
 * Loads a RASAero/OpenRocket .rse motor file and provides interpolated
 * thrust and propellant mass at any point in time.
 *
 * The .rse file is an XML-like format with an <engine> header containing
 * total and propellant weights, followed by a <data> section of
 * <eng-data t="..." f="..." m="..."/> entries.
 *
 * Usage:
 *   Motor motor;
 *   motor.loadFromFile("AeroTech_M1350W.rse");
 *   double thrust = motor.thrustAt(1.5);   // Newtons at t = 1.5 s
 *   double mass   = motor.propMassAt(1.5); // kg of propellant remaining
 */

class Motor{
public:
    double initialWeight;      // kg  — total motor weight (casing + propellant)
    double propellantWeight;   // kg  — propellant weight only
    double isp;                // s   — specific impulse
    
    /**
     * Parse the .rse file.  Returns true on success.
     * Populates initialWeight, propellantWeight, isp, and the internal
     * time/thrust/mass table.
     */
    bool loadFromFile(const std::string& filename);

    /** Thrust in Newtons at time t (seconds). Returns 0 after burn-out. */
    double thrustAt(double t) const;

    /** Remaining propellant mass in kg at time t. */
    double propMassAt(double t) const;

private:
    // Each row: [ time(s), thrust(N), propellantMass(kg) ]
    Matrix<double, Dynamic, 3> table_;

    // Helper: linearly interpolate column col at time t
    double interpolate_(double t, int col, double endValue) const;

    // Pulls a named XML attribute value from a string, e.g. attribute="value"
    static std::string extractAttribute_(const std::string& line,
                                        const std::string& attribute);
}