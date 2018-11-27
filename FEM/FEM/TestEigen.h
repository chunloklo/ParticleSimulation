#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include "Util.h"
#include <iostream>
#include <Eigen/Sparse>
#include "Point.h"
#include <vector>
#include "FEMProcedure.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>


class MatrixReplacement;
using Eigen::SparseMatrix;
namespace Eigen {
	namespace internal {
		// MatrixReplacement looks-like a SparseMatrix, so let's inherits its traits:
		template<>
		struct traits<MatrixReplacement> : public Eigen::internal::traits<Eigen::SparseMatrix<double> >
		{};
	}
}
// Example of a matrix-free wrapper from a user type to Eigen's compatible type
// For the sake of simplicity, this example simply wrap a Eigen::SparseMatrix.
class MatrixReplacement : public Eigen::EigenBase<MatrixReplacement> {
public:
	// Required typedefs, constants, and method:
	typedef double Scalar;
	typedef double RealScalar;
	typedef int StorageIndex;
	enum {
		ColsAtCompileTime = Eigen::Dynamic,
		MaxColsAtCompileTime = Eigen::Dynamic,
		IsRowMajor = false
	};
	Index rows() const { return _pointVector->size(); }
	Index cols() const { return _pointVector->size(); }
	template<typename Rhs>
	Eigen::Product<MatrixReplacement, Rhs, Eigen::AliasFreeProduct> operator*(const Eigen::MatrixBase<Rhs>& x) const {
		return Eigen::Product<MatrixReplacement, Rhs, Eigen::AliasFreeProduct>(*this, x.derived());
	}
	 //Custom API:
	VectorXd *_pointVector;
	MatrixXd *_massMatrix;
	vector<dvec2> *_forceVector;
	vector<dvec2> *_df;
	vector<Element> *_eleVector;
	vector<dmat2> *_referenceInv;
	vector<double> *_undeformedVol;
	double _delta_t;

	void attachParameters(VectorXd &pointVector,
		MatrixXd &massMatrix,
		vector<Element> &eleVector,
		vector<dmat2> &referenceInv,
		vector<double> &undeformedVol,
		double delta_t) {
		_pointVector = &pointVector;
		_eleVector = &eleVector;
		_referenceInv = &referenceInv;
		_undeformedVol = &undeformedVol;
		_massMatrix = &massMatrix;
		_delta_t = delta_t;
	}




	MatrixReplacement() : mp_mat(0) {}
	void attachMyMatrix(const SparseMatrix<double> &mat) {
		mp_mat = &mat;
	}
	const SparseMatrix<double> my_matrix() const { return *mp_mat; }
private:
	const SparseMatrix<double> *mp_mat;
};


// Implementation of MatrixReplacement * Eigen::DenseVector though a specialization of internal::generic_product_impl:
namespace Eigen {
	namespace internal {
		template<typename Rhs>
		struct generic_product_impl<MatrixReplacement, Rhs, SparseShape, DenseShape, GemvProduct> // GEMV stands for matrix-vector
			: generic_product_impl_base<MatrixReplacement, Rhs, generic_product_impl<MatrixReplacement, Rhs> >
		{
			typedef typename Product<MatrixReplacement, Rhs>::Scalar Scalar;
			template<typename Dest>
			static void scaleAndAddTo(Dest& dst, const MatrixReplacement& lhs, const Rhs& rhs, const Scalar& alpha)
			{
				// This method should implement "dst += alpha * lhs * rhs" inplace,
				// however, for iterative solvers, alpha is always equal to 1, so let's not bother about it.
				assert(alpha == Scalar(1) && "scaling is not implemented");
				EIGEN_ONLY_USED_FOR_DEBUG(alpha);
				// Here we could simply call dst.noalias() += lhs.my_matrix() * rhs,
				// but let's do something fancier (and less efficient):
				//for (Index i = 0; i < lhs.cols(); ++i)
				//	dst += rhs(i) * lhs.my_matrix().col(i);
				//dst.noalias() += (lhs.my_matrix() + lhs.my_matrix()) * rhs;
				computeForceDifferentials(*(lhs._pointVector), rhs, dst, *(lhs._eleVector), *(lhs._referenceInv), *(lhs._undeformedVol));
				//parameterize gamma
				dst *= (1.0 + 1.0 / lhs._delta_t);
				AsecondTerm(dst, rhs, *(lhs._massMatrix), lhs._delta_t);
			}
		};
	}
}


void testingEigen();