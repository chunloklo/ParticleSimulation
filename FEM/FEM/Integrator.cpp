#include <glm/glm.hpp>
#include "Point.h"
#include <vector>
#include "TestEigen.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "Config.h"

using namespace std;
using namespace Eigen;

void eulerForward(vector<Point> &pointVector, vector<dvec2> &forceVector, double delta_t) {
	for (size_t i = 0; i < pointVector.size(); i++) {
		pointVector[i].pos += pointVector[i].vel * delta_t;
		pointVector[i].vel += delta_t * forceVector[i];
	}
}

int counterDebug = 0;

void implicitEuler(vector<Point> &pointVector,
					vector<dmat2> &referenceInv, vector<double> &undeformedVolume,
					vector<Element> &eleVector, double delta_t) {

	//convert to Eigen formats
	int dims = 2;
	VectorXd posOld(pointVector.size() * dims);	
	VectorXd velOld(pointVector.size() * dims);
	VectorXd posNew(pointVector.size() * dims);
	VectorXd velNew(pointVector.size() * dims);
	VectorXd forceElastic(pointVector.size() * dims);
	VectorXd forceDamp(pointVector.size() * dims);

	//fill pos and vel
	for (size_t i = 0; i < pointVector.size(); i++) {
		posOld(i * 2) = pointVector[i].pos.x;
		posOld(i * 2 + 1) = pointVector[i].pos.y;
		velOld(i * 2) = pointVector[i].vel.x;
		velOld(i * 2 + 1) = pointVector[i].vel.y;

		posNew(i * 2) = pointVector[i].pos.x;
		posNew(i * 2 + 1) = pointVector[i].pos.y;
		velNew(i * 2) = pointVector[i].vel.x;
		velNew(i * 2 + 1) = pointVector[i].vel.y;

	}

	VectorXd dx(pointVector.size() * dims);
	VectorXd dv(pointVector.size() * dims);

	//Mass matrix
	VectorXd idVector(pointVector.size() * dims);

	idVector.fill(1);

	MatrixXd M = idVector.asDiagonal();


	//referenceInv[0][0][0] = 1;
	//referenceInv[0][0][1] = 0;
	//referenceInv[0][1][0] = 0;
	//referenceInv[0][1][1] = 1;

	//undeformedVolume[0] = 0.5;

	//velNew(0) = 0;
	//velNew(1) = 0;
	//velNew(2) = 0;
	//velNew(3) = 0;
	//velNew(4) = 0;
	//velNew(5) = 0;


	//velOld(0) = 0;
	//velOld(1) = 0;
	//velOld(2) = 0;
	//velOld(3) = 0;
	//velOld(4) = 0;
	//velOld(5) = 0;

	for (size_t i = 0; i < 20; i++) {
		//printf("Iteration %i\n", i);

		//std::string sep = "\n----------------------------------------\n";
		//IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

		//printf("referenceInv\n");
		//print2x2Matrix(referenceInv[0]);
		//printf("UndeformedVolume\n");
		//printf("%f\n", undeformedVolume[0]);
		//printf("Deltat\n");
		//printf("%f\n", delta_t);
		//printf("Mass Matrix\n");
		//std::cout << M.format(CleanFmt) << sep;
		//printf("posNew\n");
		//std::cout << posNew.format(CleanFmt) << sep;
		//printf("velNew\n");
		//std::cout << velNew.format(CleanFmt) << sep;



		
		//solve for dx
		
		//Set up A
		MatrixReplacement A;
		A.attachParameters(posNew, M, eleVector, referenceInv, undeformedVolume, delta_t);


		vector<dvec2> forceVector;

		for (size_t i = 0; i < pointVector.size(); i++) {
			forceVector.push_back(dvec2(0));
			pointVector[i].vel = vec2(0, 0);
		}
		computeElasticForces(forceVector, pointVector, eleVector, referenceInv, undeformedVolume);
		//printf("ElasticForceControl\n");
		//for (size_t i = 0; i < pointVector.size(); i++) {
		//	printf("x: %f, y: %f\n", forceVector[i].x, forceVector[i].y);
		//}

		////Set up b
		computeElasticForcesEigen(forceElastic, posNew, eleVector, referenceInv, undeformedVolume);
		//printf("ElasticForces\n");
		//std::cout << forceElastic.format(CleanFmt) << sep;


		forceDamp.fill(0);
		computeForceDifferentials(posNew, -1 * velNew, forceDamp, eleVector, referenceInv, undeformedVolume);

		//printf("forceDifferential\n");
		//std::cout << forceDamp.format(CleanFmt) << sep;
		
		//double gamma = 0;
		forceDamp *= -1 * config::gamma;

		VectorXd b(pointVector.size() * dims);

		b = 1 / delta_t * M * (velOld - velNew) + forceElastic + forceDamp;

		//printf("b\n");
		//std::cout << b.format(CleanFmt) << sep;

		//std::cout << b.format(CleanFmt) << sep;

		//int n = 10;
		//MatrixReplacement A;
		//Eigen::SparseMatrix<double> S = Eigen::MatrixXd::Random(n, n).sparseView(0.5, 1);
		//S = S.transpose()*S;
		//A.attachMyMatrix(S);
		//Eigen::VectorXd b(n), x;
		//b.setRandom();
		//Eigen::ConjugateGradient<MatrixReplacement, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner> cg;
		//cg.compute(A);
		//x = cg.solve(b);
		//std::cout << "CG:       #iterations: " << cg.iterations() << ", estimated error: " << cg.error() << std::endl;



		////solve for dx
		//VectorXd c(10), d;
		//c.setRandom();
		Eigen::ConjugateGradient<MatrixReplacement, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner> cg;
		cg.compute(A);
		dx = cg.solve(b);
		//std::cout << "CG:       #iterations: " << cg.iterations() << ", estimated error: " << cg.error() << std::endl;
		//std::cout << dx.format(CleanFmt) << sep;

		//std::cout << "#iterations:     " << cg.iterations() << std::endl;
		//std::cout << "estimated error: " << cg.error() << std::endl;
		//// update b, and solve again
		//dx = cg.solve(b);


		std::string sep = "\n----------------------------------------\n";
		IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
		//std::cout << dx.format(CleanFmt) << sep;

		////update x
		posNew += dx;
		////update v
		velNew += dx / delta_t;	
/*
		printf("dx\n");
		std::cout << dx.format(CleanFmt) << sep;*/
		//printf("VelNewAfter\n");
		//std::cout << velNew.format(CleanFmt) << sep;
	}
	std::string sep = "\n----------------------------------------\n";
	IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
	
	//printf("PosOld\n");
	//std::cout << posOld.format(HeavyFmt) << sep;
	//
	printf("count: %d\n", counterDebug++);
	printf("PosNew\n");
	std::cout << posNew.format(HeavyFmt) << sep;

	std::cout << (posOld + velNew * delta_t).format(HeavyFmt) << sep;
/*
	printf("PosNew - Old \n");
	std::cout << (posNew - posOld).format(HeavyFmt) << sep;

	printf("VelNew * deltaT\n");
	std::cout << (velNew * delta_t).format(HeavyFmt) << sep;
*/
	printf("VelNew\n");
	std::cout << velNew.format(HeavyFmt) << sep;

	computeElasticForcesEigen(forceElastic, posNew, eleVector, referenceInv, undeformedVolume);

	/*printf("ElasticForces\n");
	std::cout << forceElastic.format(HeavyFmt) << sep;*/

	forceDamp.fill(0);
	computeForceDifferentials(posNew, -1 * velNew, forceDamp, eleVector, referenceInv, undeformedVolume);

	//printf("forceDifferential\n");
	//std::cout << forceDamp.format(CleanFmt) << sep;

	forceDamp *= -1 * config::gamma;

	std::cout << (velOld + delta_t * M * (forceElastic + forceDamp)).format(HeavyFmt) << sep;

	for (size_t i = 0; i < pointVector.size(); i++) {
		pointVector[i].pos.x = posNew(2 * i);
		pointVector[i].pos.y = posNew(2 * i + 1);
		pointVector[i].vel.x = velNew(2 * i);
		pointVector[i].vel.y = velNew(2 * i + 1);
	}
}