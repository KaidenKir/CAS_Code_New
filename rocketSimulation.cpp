#include "rocketSimulation.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace std;

//constructor
rocketSimulation::rocketSimulation(const rocketConfig& config, const motor& motor) : config_(config), motor_(motor){
    // ── Default PID gains (all zero = open-loop, no correction) ───────────
    // Tune these before calling run():
    //   sim.cas().setOuterGains(Kp, Ki, Kd);
    //   sim.cas().setInnerGains(Kp, Ki, Kd);
    cas_.setOuterGains(outerKp, outerKi, outerKd);
    cas_.setInnerGains(innerKp, innerKi, innerKd);
}

void rocketSimulation::setInitalConditions(double launchAngleDeg, Vector3d perturbation, Vector3d omega){
    state_.reset();

    double launchAngleRad = launchAngleDeg * M_PI / 180.0;
    state_.attitude = eulerToQuaternion(Vector3d(perturbation[0], launchAngleRad + perturbation[1], perturbation[2]));
    state_.angularVelocity = omega;

    double motorDry = motor_.initialWeight - motor_.propellantWeight;
    state_.mass = config_.massDry + motorDry + motor_.propellantWeight;

    casOnline_ = false;
    logger_.clear();
}

pair<Vector3d, Vector3d> rocketSimulation::aeroForcesAndMoments() // TO-DO: Finish this function