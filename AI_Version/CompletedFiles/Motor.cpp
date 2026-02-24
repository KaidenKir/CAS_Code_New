#include "Motor.h"
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

// ─── Private static helper ────────────────────────────────────────────────────

string Motor::extractAttribute(const string& line, const string& attribute) {
    size_t pos = line.find(attribute + "=\"");
    if (pos == string::npos) return "";
    pos += attribute.length() + 2;          // skip past  attribute="
    size_t end = line.find("\"", pos);
    if (end == string::npos) return "";
    return line.substr(pos, end - pos);
}

// ─── File loading ─────────────────────────────────────────────────────────────

bool Motor::loadFromFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Motor: could not open " << filename << endl;
        return false;
    }

    vector<double> times, thrusts, masses;
    string line;
    bool inData = false;

    while (getline(file, line)) {
        // ── Engine header ──────────────────────────────────────────────────
        // The <engine> tag may span multiple lines, so collect until '>' found
        if (line.find("<engine") != string::npos && line.find("mfg=") != string::npos) {
            string tag = line;
            while (tag.find('>') == string::npos) {
                string next;
                if (!getline(file, next)) break;
                tag += " " + next;
            }
            string initWt = extractAttribute(tag, "initWt");
            if (!initWt.empty()) initialWeight    = stod(initWt) / 1000.0; // g → kg
            string propWt = extractAttribute(tag, "propWt");
            if (!propWt.empty()) propellantWeight = stod(propWt) / 1000.0;
            string ispStr = extractAttribute(tag, "Isp");
            if (!ispStr.empty()) isp              = stod(ispStr);
        }

        // ── Data section boundaries ────────────────────────────────────────
        if (line.find("<data>")  != string::npos) { inData = true;  continue; }
        if (line.find("</data>") != string::npos) { inData = false; continue; }

        // ── Data rows ──────────────────────────────────────────────────────
        if (inData && line.find("<eng-data") != string::npos) {
            string t = extractAttribute(line, "t");
            string f = extractAttribute(line, "f");
            string m = extractAttribute(line, "m");
            if (!t.empty() && !f.empty() && !m.empty()) {
                times.push_back(stod(t));
                thrusts.push_back(stod(f));
                masses.push_back(stod(m) / 1000.0);  // g → kg
            }
        }
    }
    file.close();

    if (times.empty()) {
        cout << "Motor: no thrust curve data in " << filename << endl;
        return false;
    }

    // Pack into Eigen matrix for fast access
    table_.resize(static_cast<int>(times.size()), 3);
    for (int i = 0; i < static_cast<int>(times.size()); i++) {
        table_(i, 0) = times[i];
        table_(i, 1) = thrusts[i];
        table_(i, 2) = masses[i];
    }
    return true;
}

// ─── Interpolation helpers ────────────────────────────────────────────────────

/**
 * Linearly interpolates column `col` of the table at time t.
 * `endValue` is returned once t exceeds the last entry.
 *
 * Linear interpolation works like this:
 *   Given two known points (t0, v0) and (t1, v1),
 *   the value at t is:  v = v0 + (t-t0)/(t1-t0) * (v1-v0)
 *   The fraction (t-t0)/(t1-t0) is called alpha (0 = at t0, 1 = at t1).
 */
double Motor::interpolate(double t, int col, double endValue) const {
    if (table_.rows() == 0) return 0.0;

    // Before the first entry — return first value
    if (t <= table_(0, 0)) return table_(0, col);

    // Past the last entry — return the specified end value
    if (t >= table_(table_.rows() - 1, 0)) return endValue;

    // Find the bracketing rows and interpolate
    for (int i = 1; i < table_.rows(); i++) {
        double t0 = table_(i-1, 0), t1 = table_(i, 0);
        if (t >= t0 && t <= t1) {
            double alpha = (t - t0) / (t1 - t0);
            return table_(i-1, col) + alpha * (table_(i, col) - table_(i-1, col));
        }
    }
    return endValue;
}

double Motor::thrustAt(double t) const {
    // Thrust goes to 0 after burn-out
    return interpolate(t, 1, 0.0);
}

double Motor::propMassAt(double t) const {
    // Propellant mass stays at last recorded value after burn-out
    return interpolate(t, 2, table_(table_.rows()-1, 2));
}
