#pragma once
#include <glm/glm.hpp>
#include "Point.h"
#include <vector>

using namespace std;
using namespace glm;

void forceDamp(vector<Point> &pointVector, vector<dvec2> &forceVector, double dampFactor);
void applyAcceleration(vector<Point> &pointVector, vector<dvec2> &forceVector, dvec2 &force);