#include "RocketState.h"

void RocketState::reset() {
    position        = Vector3d::Zero();
    velocity        = Vector3d::Zero();
    attitude        = Quaterniond(0, 0, 0, 1);
    angularVelocity = Vector3d::Zero();
    desiredAttitude = Quaterniond(0, 0, 0, 1);
    finDeflections  = Vector4d::Zero();
    finCommands     = Vector4d::Zero();
    mass            = 0.0;
    time            = 0.0;
    step            = 0;
}