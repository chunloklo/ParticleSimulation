#include <glm/glm.hpp>
#include "Point.h"
#include <vector>
#include "TestEigen.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void eulerForward(vector<Point> &pointVector, vector<dvec2> &forceVector, double delta_t) {
	for (size_t i = 0; i < pointVector.size(); i++) {
		pointVector[i].pos += pointVector[i].vel * delta_t;
		pointVector[i].vel += delta_t * forceVector[i];
	}
}

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


	for (size_t i = 0; i < 10; i++) {

		printf("Iteration %i\n", i);
		//solve for dx
		
		//Set up A
		MatrixReplacement A;
		A.attachParameters(posNew, M, eleVector, referenceInv, undeformedVolume, delta_t);

		////Set up b
		computeElasticForcesEigen(forceElastic, posNew, eleVector, referenceInv, undeformedVolume);
		forceDamp.fill(0);
		computeForceDifferentials(posNew, velNew, forceDamp, eleVector, referenceInv, undeformedVolume);
		
		double gamma = 1;
		forceDamp *= -1 * gamma;

		VectorXd b(pointVector.size() * dims);
		b = 1 / delta_t * M * (velOld - velNew) + forceElastic + forceDamp;

		std::string sep = "\n----------------------------------------\n";
		IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
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


		//std::string sep = "\n----------------------------------------\n";
		//IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
		//std::cout << dx.format(CleanFmt) << sep;

		////update x
		posNew += dx;
		////update v
		velNew += 1 / delta_t * dx;
	}
	std::string sep = "\n----------------------------------------\n";
	IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	std::cout << posNew.format(CleanFmt) << sep;
}