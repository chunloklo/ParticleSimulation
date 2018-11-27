#include <glm/glm.hpp>
#include "Point.h"
#include <vector>

using namespace std;
using namespace glm;

void forceDamp(vector<Point> &pointVector, vector<dvec2> &forceVector, double dampFactor) {
	for (size_t i = 0; i < pointVector.size(); i++) {
		forceVector[i] += -1.0 * pointVector[i].vel * dampFactor;
	}
}

void applyAcceleration(vector<Point> &pointVector, vector<dvec2> &forceVector, dvec2 &force) {
	for (size_t i = 0; i < pointVector.size(); i++) {
		forceVector[i] += force;
	}
}