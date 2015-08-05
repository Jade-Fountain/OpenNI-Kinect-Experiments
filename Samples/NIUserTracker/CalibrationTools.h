//////////////////////////////////////////////////////////////////////////////
//	  Author: Jake Fountain 2015
//    
//
//////////////////////////////////////////////////////////////////////////////

#ifndef AUTOCAL_CALIBRATION_TOOLS_H
#define AUTOCAL_CALIBRATION_TOOLS_H

#include <armadillo>
#include <math.h>

namespace autocal{

	class CalibrationTools{

		int kroneckerDelta(int i, int j);

		arma::mat33 crossMatrix(const arma::mat33& v);

		arma::mat solveWithSVD(const arma::mat& A, const arma::vec& b);

		arma::vec quaternion_from_matrix(const arma::mat33& m);

		arma::mat33 matrix_from_quaternion(const arma::vec& q);

		std::pair<arma::mat44, arma::mat44> solveHomogeneousDualSylvester(const std::vector<arma::mat44>& samplesA,const std::vector<arma::mat44>& samplesB);

	};
}
#endif
