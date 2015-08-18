//////////////////////////////////////////////////////////////////////////////
//	  Author: Jake Fountain 2015
//    
/// \file CalibrationTools.cpp
/// \brief CPP file for CalibrationTools.h
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <math.h>
#include "CalibrationTools.h"

namespace autocal{

	using utility::math::geometry::UnitQuaternion;
	using utility::math::matrix::Rotation3D;
	using utility::math::matrix::Transform3D;

	int CalibrationTools::kroneckerDelta(int i, int j){
		return (i == j) ? 1 : 0;
	}

	/*
	Returns matrix M(v) such that for any vector x, cross(v,x) = dot(M(v),x)
	*/
	arma::mat33 CalibrationTools::crossMatrix(const arma::vec3& v){
		arma::mat33 omega;
		omega <<  0 <<    -v[2] <<  v[1] << arma::endr
			  << v[2] <<     0  << -v[0] << arma::endr
			  << -v[1]<<   v[0] <<    0  << arma::endr;
		return omega;
	}

	//
	// void CalibrationTools::checkOrthonormal(M){

	// 	if M.shape[0] == 4:
	// 		if numpy.linalg.norm(M[3,:3]) != 0:
	// 			print "\n\n\n\n\n\n\nBottom row of matrix non-zero ", numpy.linalg.norm(M[3,:3]), "\n\n\n\n\n\n\n"
	// 			return False
	// 	for i in range(3):
	// 		for j in range(3):
	// 			if not (numpy.allclose(dot(M[:3,i],M[:3,j]), kroneckerDelta(i,j))):
	// 				print "\n\n\n\n\n\n\nColumn ", i, " and Column ", j, " are not orthonormal: dotprod = ", dot(M[:3,i],M[:3,j]), "\n\n\n\n\n\n\n"
	// 				return False
	// 	return True
	// }

	/*
		Returns least squares solution x to Ax = b using the psuedoinverse
	*/
	arma::mat CalibrationTools::solveWithSVD(const arma::mat& A, const arma::vec& b){
		if (A.n_rows != b.n_rows){
			throw("Problem badly formulated!");
		}
		return arma::pinv(A) * b;
	}

		/*
		solves AX=YB for X,Y and A in sampleA, B in sampleB

		Source:
		@ARTICLE{Zhuang1994, 
		author={Hanqi Zhuang and Roth, Zvi S. and Sudhakar, R.}, 
		journal={Robotics and Automation, IEEE Transactions on}, 
		title={Simultaneous robot/world and tool/flange calibration by solving homogeneous transformation equations of the form AX=YB}, 
		year={1994}, 
		month={Aug}, 
		volume={10}, 
		number={4}, 
		pages={549-554}
		}	
		*/
	std::pair<Transform3D, Transform3D> CalibrationTools::solveHomogeneousDualSylvester(const std::vector<Transform3D>& samplesA,const std::vector<Transform3D>& samplesB){

		Transform3D X,Y; std::cout << __LINE__ << std::endl;

		arma::mat combinedG; std::cout << __LINE__ << std::endl;
		arma::vec combinedC; std::cout << __LINE__ << std::endl;

		float a0 = 0; std::cout << __LINE__ << std::endl;
		float b0 = 0; std::cout << __LINE__ << std::endl;

		arma::vec3 a; std::cout << __LINE__ << std::endl;
		arma::vec3 b; std::cout << __LINE__ << std::endl;

		for (int i = 0; i < samplesA.size(); i++){
			const Transform3D& A = samplesA[i];  std::cout << __LINE__ << std::endl;
			const Transform3D& B = samplesB[i];  std::cout << __LINE__ << std::endl;
			
			//Get Quaternions for rotations
			UnitQuaternion quat_a(A.rotation());  std::cout << __LINE__ << std::endl;
			a0 = quat_a[0];  std::cout << __LINE__ << std::endl;
			a = quat_a.rows(1,3);  std::cout << __LINE__ << std::endl;
			
			UnitQuaternion quat_b(B.rotation());  std::cout << __LINE__ << std::endl;
			b0 = quat_b[0];  std::cout << __LINE__ << std::endl;
			b = quat_b.rows(1,3);  std::cout << __LINE__ << std::endl;

			//Compute G in Gw = C
			if(std::fabs(a0) < 1e-10){
				std::cout << "\n\n\n\n\nBAD SAMPLE\n\n\n\n" << std::endl;  std::cout << __LINE__ << std::endl;
				return std::pair<Transform3D, Transform3D>();  std::cout << __LINE__ << std::endl;
			}

			arma::mat G1 = a0 * arma::eye(3,3) + crossMatrix(a) + a*a.t() / a0;  std::cout << __LINE__ << std::endl;
			arma::mat G2 = -b0 * arma::eye(3,3) + crossMatrix(b) - a*b.t() / a0;  std::cout << __LINE__ << std::endl;
			
			arma::mat G = arma::join_rows(G1,G2);  std::cout << __LINE__ << std::endl;

			//Compute C in Gw = C
			arma::vec C = b - (b0/a0) * a;  std::cout << __LINE__ << std::endl;

			if (i == 0){	
				combinedG = G;  std::cout << __LINE__ << std::endl;
				combinedC = C;  std::cout << __LINE__ << std::endl;
			}else{	
				combinedG = arma::join_cols(combinedG,G);  std::cout << __LINE__ << std::endl;
				combinedC = arma::join_cols(combinedC,C);  std::cout << __LINE__ << std::endl;
			}
		}

		arma::vec w = solveWithSVD(combinedG,combinedC);  std::cout << __LINE__ << std::endl;
		
		//Compute x and y Quaternions
		UnitQuaternion x, y;  std::cout << __LINE__ << std::endl;
		y[0] = 1 / std::sqrt(1 + w[3]*w[3] + w[4]*w[4] + w[5]*w[5]);  std::cout << __LINE__ << std::endl;
		if (std::fabs(y[0])< 1e-10){
			std::cout << "\n\n\n\n\n\n\ny[0] == 0 so you need to rotate the ref base with respect to the base\n\n\n\n\n\n\n" << std::endl;  std::cout << __LINE__ << std::endl;
			return std::pair<Transform3D, Transform3D>();  std::cout << __LINE__ << std::endl;
		}
		y.rows(1,3) = y[0] * w.rows(3,5);  std::cout << __LINE__ << std::endl;
		x.rows(1,3) = y[0] * w.rows(0,2);  std::cout << __LINE__ << std::endl;
		
		float x0 = arma::dot((a/a0), x.rows(1,3)) + (b0/a0) * y[0] - arma::dot((b/a0) , y.rows(1,3));  std::cout << __LINE__ << std::endl;
		int x_sign = x0 > 0 ? 1 : -1;  std::cout << __LINE__ << std::endl;
		x[0] = x_sign * std::sqrt(1 - x[1]*x[1] - x[2]*x[2] - x[3]*x[3]);  std::cout << __LINE__ << std::endl;
		// check:
		// check = tr.quaternion_multiply(tr.quaternion_inverse(tr.quaternion_multiply(quat_a,x)), tr.quaternion_multiply(y,quat_b))

		Rotation3D Rx(x);  std::cout << __LINE__ << std::endl;
		Rotation3D Ry(y);  std::cout << __LINE__ << std::endl;

		arma::mat combinedF;  std::cout << __LINE__ << std::endl;
		arma::vec combinedD;  std::cout << __LINE__ << std::endl;

		for (int i = 0; i < samplesA.size(); i++){
			Rotation3D RA = samplesA[i].submat(0,0,2,2);  std::cout << __LINE__ << std::endl;
			arma::vec3 pA = samplesA[i].submat(0,3,2,3);  std::cout << __LINE__ << std::endl;
			arma::vec3 pB = samplesB[i].submat(0,3,2,3);  std::cout << __LINE__ << std::endl;

			arma::mat F = arma::join_rows(RA,-arma::eye(3,3));  std::cout << __LINE__ << std::endl;

			arma::vec D = Ry * pB - pA;  std::cout << __LINE__ << std::endl;

			if (i == 0){	
				combinedF = F;  std::cout << __LINE__ << std::endl;
				combinedD = D;  std::cout << __LINE__ << std::endl;
			}else{	
				combinedF = arma::join_cols(combinedF,F);  std::cout << __LINE__ << std::endl;
				combinedD = arma::join_cols(combinedD,D);  std::cout << __LINE__ << std::endl;
			}
		}
		arma::vec pxpy = solveWithSVD(combinedF,combinedD);  std::cout << __LINE__ << std::endl;
		
		X.submat(0,0,2,2) = Rx;  std::cout << __LINE__ << std::endl;
		Y.submat(0,0,2,2) = Ry;  std::cout << __LINE__ << std::endl;

		X.submat(0,3,2,3) = pxpy.rows(0,2);  std::cout << __LINE__ << std::endl;
		Y.submat(0,3,2,3) = pxpy.rows(3,5);  std::cout << __LINE__ << std::endl;

		return std::pair<Transform3D, Transform3D>(X,Y);
	}
}
