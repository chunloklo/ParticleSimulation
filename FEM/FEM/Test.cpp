#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#include "Point.h"
#include "FEMProcedure.h"

#include <glm/glm.hpp>
#include <vector>

using namespace std;

void readTestData(vector<Point> &refVector, vector<Point> &pointVector, vector<Element> &eleVector, double *dampFactor) {
	ifstream infile("./testData.txt");
	string line;

	Point *refArr = new Point[100];
	int refArrSize = 0;

	Point *pointArr = new Point[100];
	int pointArrSize = 0;

	Element *eleArr = new Element[100];
	int eleArrSize = 0;

	double dampInput;

	double kInput;
	double nuInput;

	while (getline(infile, line))
	{
		istringstream iss(line);

		while (getline(iss, line, ' ')) {
			if (line.compare("r") == 0) {
				float i;
				float j;

				getline(iss, line, ' ');
				i = strtof(line.c_str(), 0);

				getline(iss, line, ' ');
				j = strtof(line.c_str(), 0);

				refArr[refArrSize].pos = glm::vec2(i, j);
				refArrSize++;
			}

			if (line.compare("p") == 0) {
				float i;
				float j;

				getline(iss, line, ' ');
				i = strtof(line.c_str(), 0);

				getline(iss, line, ' ');
				j = strtof(line.c_str(), 0);

				pointArr[pointArrSize].pos = glm::vec2(i, j);
				pointArrSize++;
			}

			if (line.compare("e") == 0) {
				int i;
				int j;
				int k;

				getline(iss, line, ' ');
				i = stoi(line.c_str(), 0);

				getline(iss, line, ' ');
				j = stoi(line.c_str(), 0);

				getline(iss, line, ' ');
				k = stoi(line.c_str(), 0);

				eleArr[eleArrSize].i = i;
				eleArr[eleArrSize].j = j;
				eleArr[eleArrSize].k = k;

				double mu = calculateMu(kInput, nuInput);
				double lambda = calculateLambda(kInput, nuInput);
				eleArr[eleArrSize].mu = mu;
				eleArr[eleArrSize].lambda = lambda;

				eleArrSize++;
			}

			if (line.compare("damp") == 0) {
				getline(iss, line, ' ');
				dampInput = strtof(line.c_str(), 0);
			}

			if (line.compare("k") == 0) {
				getline(iss, line, ' ');
				kInput = strtof(line.c_str(), 0);
			}

			if (line.compare("nu") == 0) {
				getline(iss, line, ' ');
				nuInput = strtof(line.c_str(), 0);
			}

		}
	}

	assert(pointArrSize == refArrSize);

	for (int i = 0; i < refArrSize; i++) {
		refVector.push_back(refArr[i]);
	}

	for (int i = 0; i < pointArrSize; i++) {
		pointVector.push_back(pointArr[i]);
	}

	for (int i = 0; i < eleArrSize; i++) {
		eleVector.push_back(eleArr[i]);
	}

	*dampFactor = dampInput;
}


