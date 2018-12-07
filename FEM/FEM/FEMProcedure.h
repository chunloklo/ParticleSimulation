#pragma once
#include <glm/glm.hpp>
#include "Point.h"
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace glm;
using namespace Eigen;
void precomputation(vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol);
void computeElasticForces(vector<dvec2> &forces, vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol);

double calculateMu(double k, double nu);
double calculateLambda(double k, double nu);

void computeForceDifferentials(VectorXd &pointVector,
	const VectorXd &dx,
	VectorXd &df,
	vector<Element> &eleVector,
	vector<dmat2> &referenceInv,
	vector<double> &undeformedVol);


void computeElasticForcesEigen(VectorXd &forceVector, VectorXd &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol);
void AddSecondTerm(VectorXd &lhs, const VectorXd &rhs, MatrixXd &M, double delta_t);

void test(vector<Element> &eleVector);