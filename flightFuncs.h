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

atmosphere getAtmosphere(double altitude);

pair<Vector3d, Vector3d> calculateAerodynamics(const rocketState& tempState_Aero);

void updateCAS(double dt);

