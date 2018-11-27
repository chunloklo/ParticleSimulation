#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include "Point.h"
#include <vector>
#include "Util.h"
#include "FEMProcedure.h"

using namespace std;
using namespace glm;

void test(vector<Element> &eleVector) {
	
}

void precomputation(vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol) {
	for (size_t i = 0; i < eleVector.size(); i++) {
		Point p1 = pointVector[eleVector[i].i];
		Point p2 = pointVector[eleVector[i].j];
		Point p3 = pointVector[eleVector[i].k];

		dmat2 reference = dmat2();
		reference[0] = p1.pos - p3.pos;
		reference[1] = p2.pos - p3.pos;
		referenceInv.push_back(inverse(reference));
		undeformedVol.push_back(abs(1 / 2.0f * determinant(reference)));
	}
}

double computeTrace(dmat2 A) {
	double trace = 0;
	for (int i = 0; i < 2; i++) {
		trace += A[i][i];
	}
	return trace;
}

dmat2 calcluateStress(dmat2 deformationMatrix, Element &element) {
	dmat2 strainTensor = 0.5 * (transpose(deformationMatrix) * deformationMatrix - dmat2(1.0));

	//check if this works
	double trace = computeTrace(strainTensor);
	return deformationMatrix * (2.0 * element.mu * strainTensor  + element.lambda * trace * dmat2(1.0));
}

void computeElasticForces(vector<dvec2> &forceVector, vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol) {
	fill(forceVector.begin(), forceVector.end(), dvec2(0.0));
	for (size_t i = 0; i < eleVector.size(); i++) {
		Point p1 = pointVector[eleVector[i].i];
		Point p2 = pointVector[eleVector[i].j];
		Point p3 = pointVector[eleVector[i].k];

		dmat2 deformedShape = dmat2();
		deformedShape[0] = p1.pos - p3.pos;
		deformedShape[1] = p2.pos - p3.pos;

		dmat2 deformationMatrix = deformedShape * referenceInv[i];

		dmat2 stressTensor = calcluateStress(deformationMatrix, eleVector[i]);

		dmat2 forces = -1 * undeformedVol[i] * stressTensor * transpose(referenceInv[i]);


		forceVector[eleVector[i].i] += forces[0];
		forceVector[eleVector[i].j] += forces[1];
		forceVector[eleVector[i].k] -= (forces[0] + forces[1]);
	}

}

double calculateMu(double k, double nu) {
	return k / (2 * (1 + nu));
}

double calculateLambda(double k, double nu) {
	return k * nu / ((1 + nu) * (1 - 2 * nu));
}

dmat2 computeStressDifferentials(dmat2 F, dmat2 deltaF, Element &element) {
	dmat2 E = 0.5 * (transpose(F) * F - dmat2(1.0));
	dmat2 deltaE = 0.5 * (transpose(deltaF) * F + transpose(F) * deltaF);

	double trace = computeTrace(E);
	dmat2 deltaP = deltaF * (2 * element.mu * E + element.lambda * trace * dmat2(1.0))
		+ F * (2 * element.mu * deltaE + element.lambda * deltaE * dmat2(1.0));
	return deltaP;
}


void computeForceDifferentials(VectorXd &pointVector,
	const VectorXd &dx,
	VectorXd &df,
	vector<Element> &eleVector,
	vector<dmat2> &referenceInv,
	vector<double> &undeformedVol) {

	for (size_t i = 0; i < eleVector.size(); i++) {

		dvec2 p1 = dvec2(pointVector(eleVector[i].i * 2), pointVector(eleVector[i].i * 2 + 1));
		dvec2 p2 = dvec2(pointVector(eleVector[i].j * 2), pointVector(eleVector[i].j * 2 + 1));
		dvec2 p3 = dvec2(pointVector(eleVector[i].k * 2), pointVector(eleVector[i].k * 2 + 1));

		dmat2 deformedShape = dmat2();
		deformedShape[0] = p1 - p3;
		deformedShape[1] = p2 - p3;

		dvec2 dx1 = dvec2(dx(eleVector[i].i * 2), dx(eleVector[i].i * 2 + 1));
		dvec2 dx2 = dvec2(dx(eleVector[i].j * 2), dx(eleVector[i].j * 2 + 1));
		dvec2 dx3 = dvec2(dx(eleVector[i].k * 2), dx(eleVector[i].k * 2 + 1));

		dmat2 deltaDs = dmat2();
		deltaDs[0] = dx1 - dx3;
		deltaDs[1] = dx2 - dx3;

		dmat2 deformationMatrix = deformedShape * referenceInv[i];

		dmat2 deltaF = deltaDs * referenceInv[i];

		dmat2 deltaP = computeStressDifferentials(deformationMatrix, deltaF, eleVector[i]);

		dmat2 deltaH = -1 * undeformedVol[i] * deltaP * transpose(referenceInv[i]);

		df(eleVector[i].i * 2) += deltaH[0][0];
		df(eleVector[i].i * 2 + 1) += deltaH[0][1];
		df(eleVector[i].j * 2) += deltaH[1][0];
		df(eleVector[i].j * 2 + 1) += deltaH[1][1];
		df(eleVector[i].k * 2) -= (deltaH[0] + deltaH[1])[0];
		df(eleVector[i].k * 2 + 1) -= (deltaH[0] + deltaH[1])[1];
	}

}

void computeElasticForcesEigen(VectorXd &forceVector, VectorXd &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol) {
	forceVector.fill(0);

	for (size_t i = 0; i < eleVector.size(); i++) {
		dvec2 p1 = dvec2(pointVector(eleVector[i].i * 2), pointVector(eleVector[i].i * 2 + 1));
		dvec2 p2 = dvec2(pointVector(eleVector[i].j * 2), pointVector(eleVector[i].j * 2 + 1));
		dvec2 p3 = dvec2(pointVector(eleVector[i].k * 2), pointVector(eleVector[i].k * 2 + 1));

		dmat2 deformedShape = dmat2();
		deformedShape[0] = p1 - p3;
		deformedShape[1] = p2 - p3;

		dmat2 deformationMatrix = deformedShape * referenceInv[i];

		dmat2 stressTensor = calcluateStress(deformationMatrix, eleVector[i]);

		dmat2 forces = -1 * undeformedVol[i] * stressTensor * transpose(referenceInv[i]);

		forceVector(eleVector[i].i * 2) += forces[0][0];
		forceVector(eleVector[i].i * 2 + 1) += forces[0][1];
		forceVector(eleVector[i].j * 2 + 1) += forces[1][1];
		forceVector(eleVector[i].k * 2 + 1) -= (forces[0] + forces[1])[1];
	}

}

void AsecondTerm(VectorXd &dst, const VectorXd &rhs, MatrixXd &M, double delta_t) {
	dst = 1.0 / (delta_t * delta_t) * M * rhs;
}