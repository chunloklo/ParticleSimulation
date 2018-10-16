#pragma once
#include <glm/glm.hpp>
#include "Point.h"
#include <vector>

void precomputation(vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol);
void computeElasticForces(vector<dvec2> &forces, vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol);