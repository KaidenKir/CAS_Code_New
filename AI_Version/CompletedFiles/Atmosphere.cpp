#include "Atmosphere.h"
#include <cmath>

Atmosphere Atmosphere::at(double altitude_m) {
    Atmosphere atm;
    atm.temperature  = T0 - L * altitude_m;
    double ratio     = atm.temperature / T0;
    atm.pressure     = p0   * std::pow(ratio, 5.295);
    atm.density      = rho0 * std::pow(ratio, 4.295);
    atm.speedOfSound = std::sqrt(1.4 * 287.0 * atm.temperature);
    return atm;
}
