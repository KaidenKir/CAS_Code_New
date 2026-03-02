#include "rocketState.h"

void RocketState::reset() {
    position        = Vector3d::Zero();
    velocity        = Vector3d::Zero();
    attitude        = Quaterniond(1, 0, 0, 0);
    angularVelocity = Vector3d::Zero();
    desiredAttitude = Quaterniond(1, 0, 0, 0);
    finDeflections  = Vector4d::Zero();
    finCommands     = Vector4d::Zero();
    mass            = 0.0;
    time            = 0.0;
    step            = 0;
}