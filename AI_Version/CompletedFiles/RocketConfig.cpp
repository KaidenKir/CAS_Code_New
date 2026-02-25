#include "RocketConfig.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// ─── File loading ─────────────────────────────────────────────────────────────

bool RocketConfig::loadSimData(const string& filename) {
    // Compute derived fin geometry first
    finArea        = 0.5 * (rootChord + tipChord) * span;
    finAspectRatio = (span * span) / finArea;

    ifstream file(filename);
    if (!file.is_open()) {
        cout << "RocketConfig: could not open " << filename << endl;
        return false;
    }

    vector<double> vel, mass, lMOI, rMOI, cp, cg;
    string line;

    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;   // skip comments/blanks

        stringstream ss(line);
        string token;
        vector<double> v;

        while (getline(ss, token, ',')) {
            try { v.push_back(stod(token)); }
            catch (...) { /* skip non-numeric tokens */ }
        }

        if (v.size() == 6) {
            vel.push_back(v[0]);
            mass.push_back(v[1]);
            lMOI.push_back(v[2]);
            rMOI.push_back(v[3]);
            // Original data is in mm from some reference; convert to m and negate
            // so that positions are negative-forward from the nose tip.
            cp.push_back(-v[4] / 1000.0);
            cg.push_back(-v[5] / 1000.0);
        }
    }
    file.close();

    if (mass.empty()) {
        cout << "RocketConfig: no data found in " << filename << endl;
        return false;
    }

    // ── Remove duplicate / invalid rows ──────────────────────────────────
    // Rows where mass didn't change or CP is NaN are noise from the sim export.
    vector<bool> keep(mass.size() - 1, false);
    int keepCount = 0;
    for (int i = 1; i < (int)mass.size(); i++) {
        bool valid = (mass[i] != mass[i-1]) && !std::isnan(cp[i-1]);
        keep[i-1] = valid;
        if (valid) keepCount++;
    }

    simData_.conservativeResize(keepCount, 6);
    int row = 0;
    for (int i = 0; i < (int)mass.size() - 1; i++) {
        if (keep[i]) {
            simData_.row(row++) << vel[i], mass[i], lMOI[i], rMOI[i], cg[i], cp[i];
        }
    }
    return true;
}

// ─── MOI interpolation ────────────────────────────────────────────────────────

/**
 * The sim table is sorted by decreasing mass (propellant burns off).
 * We search for the two rows that bracket the current mass and linearly
 * interpolate the MOI values between them.
 *
 * col 1 = mass, col 2 = longitidinal MOI, col 3 = radial MOI
 */
pair<double, double> RocketConfig::getMOI(double mass) const {
    if (simData_.rows() == 0) return {0.0, 0.0};

    // Above maximum recorded mass — use first row
    if (mass >= simData_(0, 1)) return {simData_(0, 2), simData_(0, 3)};

    for (int i = 1; i < simData_.rows(); i++) {
        double m0 = simData_(i-1, 1), m1 = simData_(i, 1);
        if (mass == m1) return {simData_(i, 2), simData_(i, 3)};
        if (mass > m1 && mass < m0) {
            double alpha = (mass - m0) / (m1 - m0);
            return {lerp(alpha, simData_(i-1, 2), simData_(i, 2)),
                    lerp(alpha, simData_(i-1, 3), simData_(i, 3))};
        }
    }
    return {0.0, 0.0};
}

// ─── CG / CP interpolation ────────────────────────────────────────────────────

/**
 * CG depends on mass (propellant burn-off shifts the CG).
 * CP depends on velocity (compressibility moves the CP).
 * Each is found independently with its own linear search.
 */
pair<double, double> RocketConfig::getCGandCP(double mass, double velocity) const {
    if (simData_.rows() == 0) return {0.0, 0.0};

    // Early-out: below minimum velocity or at / above max mass → first row
    if (velocity <= simData_(0, 0) || mass >= simData_(0, 1))
        return {simData_(0, 4), simData_(0, 5)};

    // ── CP via velocity (col 0 = velocity, col 5 = CP) ──────────────────
    double cpPos = simData_(simData_.rows()-1, 5); // default: last row
    for (int i = 1; i < simData_.rows(); i++) {
        double v0 = simData_(i-1, 0), v1 = simData_(i, 0);
        if (velocity == v1) { cpPos = simData_(i, 5); break; }
        if (velocity > v0 && velocity < v1) {
            double alpha = (velocity - v0) / (v1 - v0);
            cpPos = lerp(alpha, simData_(i-1, 5), simData_(i, 5));
            break;
        }
    }

    // ── CG via mass (col 1 = mass, col 4 = CG) ───────────────────────────
    double cgPos = simData_(simData_.rows()-1, 4); // default: last row
    for (int i = 1; i < simData_.rows(); i++) {
        double m0 = simData_(i-1, 1), m1 = simData_(i, 1);
        if (mass == m1) { cgPos = simData_(i, 4); break; }
        if (mass > m0 && mass < m1) {
            double alpha = (mass - m0) / (m1 - m0);
            cgPos = lerp(alpha, simData_(i-1, 4), simData_(i, 4));
            break;
        }
    }

    return {cgPos, cpPos};
}
