#pragma once

#include "config.h"
#include <string>

using namespace std;

//Extract atribiutes from XML
string extractAttribute(const string& line, const string& attribute);

// Parse RSE File
bool parseRSEFile(const string& filename, motorConfig& motor);

// Parse MOI File
bool parseRocketFile(const string& filename, rocketConfig& config);
