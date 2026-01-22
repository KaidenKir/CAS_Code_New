#include "pidDef.h"
#include <cmath>
#include <vector>

using namespace std;

double PID(double actual, double goal, double dt, PID_vals& state) {
    if (!state.initialized) {
        state.prevActual = actual;
        state.dFiltered = 0.0;
        state.initialized = true;
        state.alpha = dt/(state.tau + dt);
    }

    double error = (goal - actual);

    if(sign(error) != sign(state.integral)) state.integral = 0.0;

    state.integral += error * dt;
    state.integral = clamp(state.integral, -state.integralLimit, state.integralLimit);

    double de_dt = -(actual - state.prevActual)/dt;
    state.dFiltered = (1.0 - state.alpha) * state.dFiltered + state.alpha * de_dt;

    double pTerm = error * state.PID_vals[0];
    double iTerm = state.integral * state.PID_vals[1];
    double dTerm = state.dFiltered * state.PID_vals[2];

    state.prevActual = actual;

    return pTerm + iTerm + dTerm;
}