#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include "Point.h"
#include <vector>

using namespace std;
using namespace glm;
void precomputation(vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol) {
	for (size_t i = 0; i < eleVector.size(); i++) {
		Point p1 = pointVector[eleVector[i].i];
		Point p2 = pointVector[eleVector[i].j];
		Point p3 = pointVector[eleVector[i].k];

		dmat2 reference = dmat2();
		reference[0] = p1.pos - p2.pos;
		reference[1] = p1.pos - p3.pos;

		referenceInv.push_back(inverse(reference));
		undeformedVol.push_back(1 / 16.0f * determinant(reference));
	}
}

double miu = 1.0;
double lambda = 1.0;

dmat2 calcluateStress(dmat2 deformationMatrix) {
	dmat2 strainTensor = (transpose(deformationMatrix) * deformationMatrix - dmat2(1.0));
	double trace = 0;
	for (int i = 0; i < 2; i++) {
		trace += strainTensor[i][i];
	}

	return deformationMatrix * (2.0 * miu * strainTensor  + lambda * trace * dmat2(1.0));
}

void computeElasticForces(vector<dvec2> &forceVector, vector<Point> &pointVector, vector<Element> &eleVector, vector<dmat2> &referenceInv, vector<double> &undeformedVol) {
	fill(forceVector.begin(), forceVector.end(), dvec2(0.0));
	for (size_t i = 0; i < eleVector.size(); i++) {
		Point p1 = pointVector[eleVector[i].i];
		Point p2 = pointVector[eleVector[i].j];
		Point p3 = pointVector[eleVector[i].k];

		dmat2 deformedShape = dmat2();
		deformedShape[0] = p1.pos - p2.pos;
		deformedShape[1] = p1.pos - p3.pos;


		dmat2 deformationMatrix = deformedShape * referenceInv[i];

		dmat2 stressTensor = calcluateStress(deformationMatrix);
		
		dmat2 forces = -1 * undeformedVol[i] * stressTensor * transpose(deformedShape);
		
		forceVector[eleVector[i].i] += forces[0];
		forceVector[eleVector[i].j] += forces[1];
		forceVector[eleVector[i].k] -= (forces[0] + forces[1]);


	}

}