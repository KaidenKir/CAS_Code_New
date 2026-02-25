#include "FlightLogger.h"
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;

void FlightLogger::record(const Sample& s) {
    time_        .push_back(s.time);
    posX_        .push_back(s.positionX);
    posY_        .push_back(s.positionY);
    posZ_        .push_back(s.positionZ);
    velMag_      .push_back(s.velocityMag);
    velVert_     .push_back(s.verticalVelocity);
    velHoriz_    .push_back(s.horizontalVelocity);
    accelMag_    .push_back(s.accelerationMag);
    roll_        .push_back(s.roll);
    pitch_       .push_back(s.pitch);
    yaw_         .push_back(s.yaw);
    rollRate_    .push_back(s.rollRate);
    pitchRate_   .push_back(s.pitchRate);
    yawRate_     .push_back(s.yawRate);
    thrust_      .push_back(s.thrust);
    drag_        .push_back(s.drag);
    mass_        .push_back(s.mass);
    reqRollRate_ .push_back(s.reqRollRate);
    reqPitchRate_.push_back(s.reqPitchRate);
    reqYawRate_  .push_back(s.reqYawRate);
    reqRollCtrl_ .push_back(s.reqRollCtrl);
    reqPitchCtrl_.push_back(s.reqPitchCtrl);
    reqYawCtrl_  .push_back(s.reqYawCtrl);
    fin1_        .push_back(s.fin1);
    fin2_        .push_back(s.fin2);
    fin3_        .push_back(s.fin3);
    fin4_        .push_back(s.fin4);
    mach_        .push_back(s.machNumber);
    dynQ_        .push_back(s.dynamicPressure);
}

void FlightLogger::clear() {
    // Clear all vectors back to empty
    time_.clear(); posX_.clear(); posY_.clear(); posZ_.clear();
    velMag_.clear(); velVert_.clear(); velHoriz_.clear();
    accelMag_.clear();
    roll_.clear(); pitch_.clear(); yaw_.clear();
    rollRate_.clear(); pitchRate_.clear(); yawRate_.clear();
    thrust_.clear(); drag_.clear(); mass_.clear();
    reqRollRate_.clear(); reqPitchRate_.clear(); reqYawRate_.clear();
    reqRollCtrl_.clear(); reqPitchCtrl_.clear(); reqYawCtrl_.clear();
    fin1_.clear(); fin2_.clear(); fin3_.clear(); fin4_.clear();
    mach_.clear(); dynQ_.clear();
}

bool FlightLogger::saveCSV(const string& filename) const {
    ofstream file(filename);
    if (!file.is_open()) {
        cout << "FlightLogger: could not open " << filename << endl;
        return false;
    }

    // Header
    file << "time,positionX,positionY,positionZ,"
         << "velocity_magnitude,vertical_velocity,horizontal_velocity,"
         << "acceleration_magnitude,"
         << "roll,pitch,yaw,"
         << "roll_rate,pitch_rate,yaw_rate,"
         << "thrust,drag,mass,"
         << "req_roll_rate,req_pitch_rate,req_yaw_rate,"
         << "req_roll_ctrl,req_pitch_ctrl,req_yaw_ctrl,"
         << "fin1,fin2,fin3,fin4,"
         << "mach_number,dynamic_pressure\n";

    // Rows
    size_t n = time_.size();
    file << fixed << setprecision(6);
    for (size_t i = 0; i < n; i++) {
        file << time_[i]        << ',' << posX_[i]        << ',' << posY_[i]        << ',' << posZ_[i]        << ','
             << velMag_[i]      << ',' << velVert_[i]     << ',' << velHoriz_[i]    << ','
             << accelMag_[i]    << ','
             << roll_[i]        << ',' << pitch_[i]       << ',' << yaw_[i]         << ','
             << rollRate_[i]    << ',' << pitchRate_[i]   << ',' << yawRate_[i]     << ','
             << thrust_[i]      << ',' << drag_[i]        << ',' << mass_[i]        << ','
             << reqRollRate_[i] << ',' << reqPitchRate_[i]<< ',' << reqYawRate_[i]  << ','
             << reqRollCtrl_[i] << ',' << reqPitchCtrl_[i]<< ',' << reqYawCtrl_[i] << ','
             << fin1_[i]        << ',' << fin2_[i]        << ',' << fin3_[i]        << ',' << fin4_[i] << ','
             << mach_[i]        << ',' << dynQ_[i]        << '\n';
    }
    file.close();
    cout << "FlightLogger: saved " << n << " samples to " << filename << endl;
    return true;
}