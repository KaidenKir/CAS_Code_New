#include "flightFuncs.h"

#include <iostream>
#include <vector>
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"
#include <fstream>
#include <unordered_map>
#include <cmath>
#include <ctime>
#include <mutex>
#include <functional>
#include <random>
#include <iomanip>
#include <string>

using namespace std;
using namespace Eigen;

// Atmospheric model (simple exponential)
atmosphere getAtmosphere(double altitude) {
    atmosphere atm;
    
    // Sea level conditions
    const double T0 = 288.0; //
    const double p0 = 1.012e5; // Pa
    const double rho0 = 1.225; // kg/m^3
    
    atm.temperature = T0 - 6.455 * altitude/1000;
    atm.pressure = p0 * pow(atm.temperature/T0, 5.295);
    atm.density = rho0 * pow(atm.temperature/T0, 4.295);
    atm.speedOfSound = sqrt(1.4 * 287 * atm.temperature);
    
    return atm;
}

pair<Vector3d, Vector3d> calculateAerodynamics(const rocketState& tempState_Aero) {
    if(tempState_Aero.velocity.hasNaN()) {
        cout << "ERROR: Input velocity to aero is NaN!" << endl;
        cout << "  tempState_Aero time: " << tempState_Aero.time << endl;
        cout << "  Velocity: ";
        for(int i = 0; i < 3; i++){
            cout << tempState_Aero.velocity(i) << ", ";
        }
        cout << endl;
        return {Vector3d::Zero(), Vector3d::Zero()};
    }
    
    atmosphere atm = getAtmosphere(tempState_Aero.position[2]);

    if(std::isinf(atm.density) || std::isnan(atm.density)) {
        cout << "ERROR: Atmosphere density is invalid!" << endl;
        cout << "  Altitude: " << tempState_Aero.position[2] << endl;
        cout << "  Density: " << atm.density << endl;
        return {Vector3d::Zero(), Vector3d::Zero()};
    }
    
    Vector3d force = Vector3d::Zero();
    Vector3d moment = Vector3d::Zero();
    
    double speed = tempState_Aero.velocity.norm();
    if (speed < 0.1) return {force, moment}; // Avoid division by zero
    
    Vector3d velocity_unit = tempState_Aero.velocity.normalized();
    double dynamic_pressure = 0.5 * atm.density * speed * speed;

    Vector3d velocityBodyFrame = tempState_Aero.attitude.conjugate()._transformVector(tempState_Aero.velocity);

    // calculate aoa 
    Vector3d rocketAxisWorld = tempState_Aero.attitude._transformVector(Vector3d(0, 0, 1));
    double cosAlphaTotal = velocity_unit.dot(rocketAxisWorld);
    cosAlphaTotal = clamp(cosAlphaTotal, -1.0, 1.0);
    double alphaTotal = acos(cosAlphaTotal);
    //force (always opposite to velocity)
    Vector3d drag = -dynamic_pressure * config.referenceArea * config.cd * velocity_unit;
    force += drag;

    auto [cg, cp] = interpolateCGCP(config, tempState_Aero.mass, tempState_Aero.velocity.norm());
    double stabilityMarginBody = -(cg - cp);
    //cout << stabilityMarginBody << endl;
    
    if (fabs(alphaTotal) > 1e-6) {            
        // Normal forces in body-fixed coordinates
        Vector3d normalDirection = rocketAxisWorld.cross(velocity_unit);
        double normalDirectionMagnitude = normalDirection.norm();
        
        if (normalDirectionMagnitude > 1e-6) {
            normalDirection /= normalDirectionMagnitude;
            
            // Normal force magnitude (using small angle approximation for Cn)
            double Cn = config.Cn_alpha * sin(alphaTotal);
            double normalForceMagnitude = dynamic_pressure * config.referenceArea * Cn;
            
            // Apply normal force at center of pressure
            Vector3d normalForceWorld = normalForceMagnitude * normalDirection;
            force += normalForceWorld;
            if(force.hasNaN()){
                cout << "force NaN 1" << endl;
            }

            // STABILITY MOMENT: Force acts at CP, create moment about CG
            
            // Moment = r × F (where r is from CG to CP)
            Vector3d normalForceBody = tempState_Aero.attitude.conjugate()._transformVector(normalForceWorld);    
            Vector3d stabilityMomentBody = Vector3d(0, 0, stabilityMarginBody).cross(normalForceBody);
            if(normalForceBody.norm() > 200){
                cout << "normalForceBody: " << normalForceBody[0] << ", " << normalForceBody[1] << ", " << normalForceBody[2] << endl;
                cout << "stabilityMomentBody: " << stabilityMomentBody[0] << ", " << stabilityMomentBody[1] << ", " << stabilityMomentBody[2] << endl;
                cout << "angularVelocity: " << state.angularVelocity[0] << ", " << state.angularVelocity[1] << ", " << state.angularVelocity[2] << endl << endl;
            } 
            moment += stabilityMomentBody;
        }
    }

    if(checkIfFinite(alphaTotal, "alphaTotal"));
    if(checkIfFinite(cosAlphaTotal, "cosAlphaTotal"));
    if(checkIfFinite(dynamic_pressure, "dynamic_pressure"));
    if(checkIfFinite(speed, "speed"));
    if(checkIfFinite(config.Cn_alpha, "config.Cn_alpha"));
    if(checkIfFinite(config.referenceArea, "config.referenceArea"));

    if(speed > 1e-6){
        double canardSpan = config.span;
        double canardChord = config.finArea / canardSpan;

        double canardAeroCenter = -(config.finLocation + 0.25 * canardChord) - cg;
        double canardAeroSpan = (config.diameter * 0.5) + (canardSpan * 0.5);

        if(checkIfFinite(canardSpan, "canardSpan"));
        if(checkIfFinite(canardChord, "canardChord"));
        if(checkIfFinite(canardAeroCenter, "canardAeroCenter"));
        if(checkIfFinite(canardAeroSpan, "canardAeroSpan"));

        vector<Vector3d> canardPositionsBody = {
            Vector3d(0, canardAeroSpan, canardAeroCenter),      // +Y
            Vector3d(0, -canardAeroSpan, canardAeroCenter),     // -Y
            Vector3d(canardAeroSpan, 0, canardAeroCenter),      // +X
            Vector3d(-canardAeroSpan, 0, canardAeroCenter),     // -X
        };
        vector<Vector3d> canardSpanVectors = {
            Vector3d(0, 1, 0),       // +Y
            Vector3d(0, -1, 0),      // +Y
            Vector3d(1, 0, 0),       // +X
            Vector3d(-1, 0, 0),      // -X
        };

        for(int i = 0; i < 4; i++){
            double deflectionRad = tempState_Aero.finDeflections[i]*M_PI/180;
            Vector3d canardSpanVector = canardSpanVectors[i];
            Vector3d canardPosition = canardPositionsBody[i];

            // calculate canard local velocity using:
            // v_loc = v_moving_frame + omega X r
            Vector3d localVelocityBody = velocityBodyFrame + tempState_Aero.angularVelocity.cross(canardPosition);
            if(tempState_Aero.angularVelocity.norm() > 500){
                cout << "step: " << tempState_Aero.step << " | Fin: " << i << endl;
                cout << "Position: " << canardPosition(0) << ", " << canardPosition(1) << ", " << canardPosition(2) << endl;
                cout << "Angular Velocity: " << tempState_Aero.angularVelocity(0) << ", " << tempState_Aero.angularVelocity(1) << ", " << tempState_Aero.angularVelocity(2) << endl;
                cout << "Velocity Body Frame: " << velocityBodyFrame(0) << ", " << velocityBodyFrame(1) << ", " << velocityBodyFrame(2) << endl;
                cout << "Local Velocity: " << localVelocityBody(0) << ", " << localVelocityBody(1) << ", " << localVelocityBody(2) << endl << endl;
            }

            double localSpeed = localVelocityBody.norm();
            double localDynamicPressure = 0.5 * atm.density * localSpeed * localSpeed;
            if(localDynamicPressure > 200000){
                cout << "step: " << tempState_Aero.step << " | Fin: " << i << endl;
                cout << "Altitude: " << tempState_Aero.position[2] << " | Speed: " << speed << endl;
                cout << "Local Speed: " << localSpeed << endl;
                cout << "Atm Density: " << atm.density << endl;
                cout << "Dynamic Pressure: " << localDynamicPressure << endl;
                cout << "Local Velocity: " << localVelocityBody(0) << ", " << localVelocityBody(1) << ", " << localVelocityBody(2) << endl << endl;
            }

            if(checkIfFinite(localSpeed, "localSpeed")) continue;
            if(checkIfFinite(localDynamicPressure, "localDynamicPressure")) continue;

            // find lift direction
            // v_local X span_vector 
            Vector3d liftDirection = localVelocityBody.cross(canardSpanVector);

            if(!liftDirection.hasNaN()){
                liftDirection = liftDirection / liftDirection.norm();
            }else{
                cout << "liftDirection has NaN" << endl;
            }
            
            // compute local AOA
            // we remove the velocity parralel to the span vector with V_local - (V_local.dot(canardSpan))*canardSpan
            Vector3d velocityInCanardPlane = localVelocityBody - (localVelocityBody.dot(canardSpanVector)) * canardSpanVector;

            if(checkIfFinite(velocityInCanardPlane, "velocityInCanardPlane")) continue;
            
            // for AOA calculation, we care about the Z component of this
            double axialComponent = velocityInCanardPlane(2);

            // calculate the perpendicular direction of the fin
            Vector3d perpDirection = Vector3d::UnitZ().cross(canardSpanVector);
            if (perpDirection.norm() < 1e-8) {
                perpDirection = Vector3d::UnitX().cross(canardSpanVector);
            }

            if(checkIfFinite(perpDirection, "perpDirection")) continue;

            perpDirection.normalize();
            double perpComponent = velocityInCanardPlane.dot(perpDirection);
            if(checkIfFinite(perpComponent, "perpComponent")) continue;

            double geometricAOA = atan2(perpComponent, axialComponent);
            if(checkIfFinite(geometricAOA, "geometricAOA")) continue;

            // compute effective AOA
            // add the canard angle with the geometric AOA
            double effectiveAOA = geometricAOA + deflectionRad;

            // flip based on which way produces positive lift for positive AOA
            // For a canard, positive deflection (trailing edge down) should create upward lift
            if (effectiveAOA * liftDirection.dot(perpDirection) < 0) {
                liftDirection = -liftDirection;
            }

            // calculate Cl
            double Cl_alpha = 0.0;
            double AR = config.finAspectRatio;
            double M = localSpeed / atm.speedOfSound;
            if(M < 0.8){
                double beta = sqrt(1-M*M);
                Cl_alpha = (2*M_PI*AR)/(2 + sqrt(4 + (AR*AR)*(1 - M*M)/(beta*beta)));
                //cout << "mach > speed @ step: " << tempState_Aero.step << endl;
            }else if(M < 1.2){
                double M_est = 0.9;
                double beta = sqrt(1-M_est*M_est);
                Cl_alpha = (2*M_PI*AR)/(2 + sqrt(4 + (AR*AR)*(1 - M_est*M_est)/(beta*beta)));
            }else{
                Cl_alpha = (4*AR)/sqrt(M*M - 1);
                if(Cl_alpha > 50){
                    cout << "mach > 1 @ step: " << tempState_Aero.step << " | fin: " << i << endl;
                    cout << "mach: " << M << endl;
                    cout << "M * M - 1: " << M*M - 1 << endl;
                    cout << "sqrt(^^): " << sqrt(M*M - 1) << endl;
                    cout << "Cl_alpha: " << Cl_alpha << endl << endl;
                }
            }
            if(checkIfFinite(Cl_alpha, "Cl_alpha")) continue;

            double Cl = Cl_alpha * effectiveAOA;
            if(checkIfFinite(Cl, "Cl")) continue;
            if(Cl > 12){
                cout << "CL_alpha: " << Cl_alpha << endl;
                cout << "Effective AoA: " << effectiveAOA << endl;
                cout << "Cl: " << Cl << endl << endl;
            }

            // compute canard lift force
            Vector3d canardLiftBody = localDynamicPressure * config.finArea * Cl * liftDirection;
            if(checkIfFinite(canardLiftBody, "canardLiftBody")) continue;
            if(fabs(canardLiftBody.norm()) > 1000){
                cout << "step: " << tempState_Aero.step << " | Fin: " << i << endl;
                cout << "Coeficient of Lift: " << Cl << endl;
                cout << "Local Dynamic Pressure: " << localDynamicPressure << endl;
                cout << "Lift Direction: " << liftDirection(0) << ", " << liftDirection(1) << ", " << liftDirection(2) << endl;
                cout << "Total Lift: " << canardLiftBody(0) << ", " << canardLiftBody(1) << ", " << canardLiftBody(2) << endl << endl;
            }

            // compute drag forces
            // induced drag
            double finEfficiency = 0.85;
            double Cd_induced = (AR > 0.0) ? (Cl * Cl) / (M_PI * AR * finEfficiency) : 0.0;

            // wave drag
            double Cd_wave = 0.0;
            if(M >= 0.8) Cd_wave = 0.5 * (M - 0.8) * (M - 0.8);

            // form drag
            double Cd_formDrag = 0.01;

            // total drag coefficient
            double Cd = Cd_induced + Cd_wave + Cd_formDrag;

            // apply drag to oposite local velocity
            Vector3d canardDragBody = localDynamicPressure * config.finArea * Cd * -localVelocityBody.normalized();

            // total canard force (body frame)
            Vector3d totalCanardForceBody = canardDragBody + canardLiftBody;
            /*cout << "Lift: " << canardLiftBody(0) << ", " << canardLiftBody(1) << ", " << canardLiftBody(2) << endl;
            cout << "Drag: " << canardDragBody(0) << ", " << canardDragBody(1) << ", " << canardDragBody(2) << endl;
            cout << "Total Force: " << totalCanardForceBody(0) << ", " << totalCanardForceBody(1) << ", " << totalCanardForceBody(2) << endl << endl;*/

            // calculate canard moment in body frame
            Vector3d totalCanardMomentBody = canardPosition.cross(totalCanardForceBody);
            /*cout << "Position: " << canardPosition(0) << ", " << canardPosition(1) << ", " << canardPosition(2) << endl;
            cout << "Total Force: " << totalCanardForceBody(0) << ", " << totalCanardForceBody(1) << ", " << totalCanardForceBody(2) << endl;
            cout << "Total Moment: " << totalCanardMomentBody(0) << ", " << totalCanardMomentBody(1) << ", " << totalCanardMomentBody(2) << endl << endl;*/

            // add to moment and force
            moment += totalCanardMomentBody;
            force += tempState_Aero.attitude._transformVector(totalCanardForceBody);

            checkIfFinite(force, "force");
            checkIfFinite(moment, "moment");
        } 
    }
    return {force, moment};
}

void updateCAS(double dt) {
    PID_state.time_since_outer_update += dt;
    PID_state.time_since_inner_update += dt;

    // outer loop commands
    if(PID_state.time_since_outer_update >= PID_state.outer_dt){
        Quaterniond qInverse = state.attitude.conjugate();
        Quaterniond qError = qInverse * state.desiredAttitude;
        Vector3d attitudeError = quaternionToAngles(qError);
        //cout << "yaw error: " << attitudeError[0] << " | pitch error: " << attitudeError[1] << " | roll error: " << attitudeError[2] << endl;

        PID_state.yaw_rate = PID(attitudeError[0], 0.0, PID_state.outer_dt, yaw_outer_pid);
        PID_state.pitch_rate = PID(attitudeError[1], 0.0, PID_state.outer_dt, pitch_outer_pid);
        PID_state.roll_rate = PID(attitudeError[2], 0.0, PID_state.outer_dt, roll_outer_pid);

        PID_state.time_since_outer_update = 0.0;
    }

    // inner loop  commands
    if(PID_state.time_since_inner_update >= PID_state.inner_dt){    
        double yaw_angularVelocity = state.angularVelocity[2];
        double pitch_angularVelocity = state.angularVelocity[1];
        double roll_angularVelocity = state.angularVelocity[0];
        
        PID_state.yaw_controll = PID(yaw_angularVelocity, PID_state.yaw_rate, PID_state.inner_dt, yaw_inner_pid);
        PID_state.pitch_controll = PID(pitch_angularVelocity, PID_state.pitch_rate, PID_state.inner_dt, pitch_inner_pid);
        PID_state.roll_controll = PID(roll_angularVelocity, PID_state.roll_rate, PID_state.inner_dt, roll_inner_pid);
        state.finDeflections[0] = clamp((PID_state.pitch_controll + PID_state.roll_controll)*180/M_PI, -config.maxFinDeflection, config.maxFinDeflection);  // +Y fin
        state.finDeflections[1] = clamp((PID_state.yaw_controll + PID_state.roll_controll)*180/M_PI, -config.maxFinDeflection, config.maxFinDeflection);   // +X fin
        state.finDeflections[2] = clamp((-PID_state.pitch_controll + PID_state.roll_controll)*180/M_PI, -config.maxFinDeflection, config.maxFinDeflection); // -Y fin
        state.finDeflections[3] = clamp((-PID_state.yaw_controll + PID_state.roll_controll)*180/M_PI, -config.maxFinDeflection, config.maxFinDeflection);  // -X fin
        
        PID_state.time_since_inner_update = 0.0;
    }
} 