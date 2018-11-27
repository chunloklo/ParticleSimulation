#pragma once
#include <glm/glm.hpp>
#include "Point.h"
#include <vector>

using namespace std;

void eulerForward(vector<Point> &pointVector, vector<dvec2> &forceVector, double delta_t);

void implicitEuler(vector<Point> &pointVector,
	vector<dmat2> &referenceInv, vector<double> &undeformedVolume,
	vector<Element> &eleVector, double delta_t);