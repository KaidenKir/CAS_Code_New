#pragma once
#include <string>
#include <iostream>
#include <cmath>
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/Geometry"

using namespace Eigen;

// ─────────────────────────────────────────────
//  Scalar helpers
// ─────────────────────────────────────────────

// Returns the sign of x: +1, -1, or 0
inline double sign(double x) {
    if (x > 0) return  1.0;
    if (x < 0) return -1.0;
    return 0.0;
}

// Prints a warning and returns true if the double is NaN or Inf
inline bool checkIfFinite(double var, const std::string& varName) {
    if (std::isnan(var)) {
        std::cout << varName << " is NaN" << std::endl;
        return true;
    } else if (std::isinf(var)) {
        std::cout << varName << " is Infinity" << std::endl;
    }
    return false;
}

// Overload for 3D vectors — checks each component
inline bool checkIfFinite(const Vector3d& var, const std::string& varName) {
    if (var.hasNaN()) {
        std::cout << varName << " has NaN" << std::endl;
        return true;
    } else if (!var.allFinite()) {
        std::cout << varName << " has Infinity" << std::endl;
    }
    return false;
}

// ─────────────────────────────────────────────
//  Quaternion helpers
// ─────────────────────────────────────────────

/**
 * Converts a quaternion to an axis-angle vector.
 *
 * A quaternion q = (w, x, y, z) encodes a rotation of angle θ around a unit
 * axis n as:  q = (cos(θ/2),  n·sin(θ/2))
 *
 * The returned vector has magnitude θ and points along n (the rotation axis).
 * Near zero rotation (θ → 0) we use the limit  θ/sin(θ/2) → 2  to avoid ÷0.
 */
inline Vector3d quaternionToAngles(const Quaterniond& q) {
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    double theta = 2.0 * acos(std::clamp(w, -1.0, 1.0));
    double sin_half_theta = sqrt(x*x + y*y + z*z);

    if (theta < 1e-6)
        return 2.0 * Vector3d(x, y, z);

    return (theta / sin_half_theta) * Vector3d(x, y, z);
}

/**
 * Builds a quaternion from Euler angles (roll, pitch, yaw) in radians.
 * Rotation order: yaw → pitch → roll  (ZYX extrinsic / XYZ intrinsic).
 */
inline Quaterniond eulerToQuaternion(const Vector3d& rpy) {
    return AngleAxisd(rpy[2], Vector3d::UnitZ())
         * AngleAxisd(rpy[1], Vector3d::UnitY())
         * AngleAxisd(rpy[0], Vector3d::UnitX());
}

/**
 * Rotates a 3D vector by a quaternion using the Rodrigues-style formula.
 *   v' = 2(u·v)u + (s²-u·u)v + 2s(u×v)
 * where u = vector part of q, s = scalar part.
 */
inline Vector3d rotateVectorByQuaternion(const Vector3d& vec, const Quaterniond& q) {
    Vector3d u(q.x(), q.y(), q.z());
    double   s = q.w();
    return 2.0 * u.dot(vec) * u
         + (s*s - u.dot(u)) * vec
         + 2.0 * s * u.cross(vec);
}

/**
 * Scales all four components of a quaternion by a scalar.
 * Useful for quaternion-derivative arithmetic (does NOT produce a unit quaternion).
 */
inline Quaterniond scaleQuaternion(const Quaterniond& q, double s) {
    return Quaterniond(q.w()*s, q.x()*s, q.y()*s, q.z()*s);
}
