#pragma once

/**
 * Atmosphere
 * ----------
 * Simple International Standard Atmosphere (ISA) model using a linear
 * temperature lapse rate and hydrostatic pressure equations.
 *
 * Call  Atmosphere::at(altitude_m)  to get conditions at any altitude.
 */
class Atmosphere {
public:
    double density;       // kg/m³
    double pressure;      // Pa
    double temperature;   // K
    double speedOfSound;  // m/s

    /**
     * Computes atmospheric properties at the given altitude above sea level.
     *
     * The model assumes:
     *   T(h) = T0 - L·h          (linear lapse)
     *   p(h) = p0·(T/T0)^5.295   (hydrostatic)
     *   ρ(h) = ρ0·(T/T0)^4.295   (ideal gas)
     *   a(h) = √(γ·R·T)          (speed of sound, γ=1.4, R=287 J/kg·K)
     */
    static Atmosphere at(double altitude_m);

private:
    // Sea-level constants
    static constexpr double T0   = 288.0;    // K
    static constexpr double p0   = 1.012e5;  // Pa
    static constexpr double rho0 = 1.225;    // kg/m³
    static constexpr double L    = 6.455e-3; // K/m  (lapse rate)
};
