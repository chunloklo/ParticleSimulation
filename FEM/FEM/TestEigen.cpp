#include <iostream>
#include <Eigen/Sparse>
#include "Point.h"
#include <vector>
#include "FEMProcedure.h"

#include "TestEigen.h"

using namespace Eigen;
using namespace std;


void testingEigen() {
	//typedef Eigen::Triplet<double> T;
	//std::vector<T> tripletList;
	//tripletList.reserve(9);
	//tripletList.push_back(T(0, 0, 2));
	//tripletList.push_back(T(1, 0, -1));
	//tripletList.push_back(T(0, 1, -1));
	//tripletList.push_back(T(1, 1, 2));
	//tripletList.push_back(T(2, 1, -1));
	//tripletList.push_back(T(1, 2, -1));
	//tripletList.push_back(T(2, 2, 2));

	////for (size_t i = 0; i < 100; i++)
	////{
	////	tripletList.push_back(T(i, i, i));
	////}



	//int n = 10;
	//VectorXd x(n), b(n);
	//SparseMatrix<double> A(n, n);
	//A.setFromTriplets(tripletList.begin(), tripletList.end());
	//b(0) = 1;
	//b(1) = 2;
	//b(2) = 3;

	//ConjugateGradient<SparseMatrix<double>, Lower | Upper> cg;
	//cg.compute(A);
	//x = cg.solve(b);
	//std::cout << "#iterations:     " << cg.iterations() << std::endl;
	//std::cout << "estimated error: " << cg.error() << std::endl;
	//// update b, and solve again
	//x = cg.solve(b);


	//std::string sep = "\n----------------------------------------\n";
	//IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	//std::cout << x.format(CleanFmt) << sep;

	////A.setFromTriplets(tripletList.begin(), tripletList.end());
	////b(0) = 1;
	////b(1) = 2;
	////b(2) = 3;

	//MatrixReplacement Asub;
	////Asub.attachMyMatrix(A);
	////Eigen::ConjugateGradient<MatrixReplacement, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner> cgsub;
	////cgsub.compute(Asub);
	////x = cgsub.solve(b);
	////std::cout << "CG:       #iterations: " << cgsub.iterations() << ", estimated error: " << cgsub.error() << std::endl;
	////std::cout << x.format(CleanFmt) << sep;

	MatrixReplacement Asub;
	int n = 10;
	Eigen::SparseMatrix<double> S = Eigen::MatrixXd::Random(n, n).sparseView(0.5, 1);
	S = S.transpose()*S;
	Asub.attachMyMatrix(S);
	Eigen::VectorXd b(n), x;
	b.setRandom();
	Eigen::ConjugateGradient<MatrixReplacement, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner> cg;
	cg.compute(Asub);
	x = cg.solve(b);
	std::cout << "CG:       #iterations: " << cg.iterations() << ", estimated error: " << cg.error() << std::endl;

	while (true) {

	}

	return;
}
