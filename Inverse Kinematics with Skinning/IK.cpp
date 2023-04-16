#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

enum IKMethods { tikhonovIK, pseudoinverseIK, transposeIK, dlsIK};
const IKMethods ikMethods = dlsIK;

namespace
{

	// Converts degrees to radians.
	template<typename real>
	inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

	template<typename real>
	Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
	{
		Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
		Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
		Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

		switch (order)
		{
		case RotateOrder::XYZ:
			return RZ * RY * RX;
		case RotateOrder::YZX:
			return RX * RZ * RY;
		case RotateOrder::ZXY:
			return RY * RX * RZ;
		case RotateOrder::XZY:
			return RY * RZ * RX;
		case RotateOrder::YXZ:
			return RZ * RX * RY;
		case RotateOrder::ZYX:
			return RX * RY * RZ;
		}
		assert(0);
	}

	// Performs forward kinematics, using the provided "fk" class.
	// This is the function whose Jacobian matrix will be computed using adolc.
	// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
	//   IKJointIDs is an array of integers of length "numIKJoints"
	// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
	// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
	template<typename real>
	void forwardKinematicsFunction(
		int numIKJoints, const int* IKJointIDs, const FK& fk,
		const std::vector<real>& eulerAngles, std::vector<real>& handlePositions)
	{
		// Students should implement this.
		// The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
		// The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
		// Then, implement the same algorithm into this function. To do so,
		// you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
		// 
		// Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
		// It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
		// so that code is only written once. We considered this; but it is actually not easily doable.
		// If you find a good approach, feel free to document it in the README file, for extra credit.

		// compute local transformation
		vector<Mat3<real>> localRotations;
		vector<Vec3<real>> localTranslations;
		for (int i = 0; i < fk.getNumJoints(); ++i) 
		{
			// rotation
			Vec3<real> eulerAngle(eulerAngles[3 * i + 0], eulerAngles[3 * i + 1], eulerAngles[3 * i + 2]);
			Mat3<real> eulerAngleR = Euler2Rotation(eulerAngle.data(), fk.getJointRotateOrder(i));
			Vec3<real> jointOrient(fk.getJointOrient(i)[0], fk.getJointOrient(i)[1], fk.getJointOrient(i)[2]);
			Mat3<real> jointOrientR = Euler2Rotation(jointOrient.data(), fk.getJointRotateOrder(i));
			localRotations.push_back(jointOrientR * eulerAngleR);

			// translation
			Vec3<real> jointRestT(fk.getJointRestTranslation(i)[0], fk.getJointRestTranslation(i)[1], fk.getJointRestTranslation(i)[2]);
			localTranslations.push_back(jointRestT);
		}

		// compute global transformation
		vector<Mat3<real>> globalRotations;
		vector<Vec3<real>> globalTranslations;
		for (int i = 0; i < fk.getNumJoints(); ++i)
		{
			int childIdx, parentIdx;
			childIdx = fk.getJointUpdateOrder(i);
			parentIdx = fk.getJointParent(childIdx);

			if (parentIdx == -1)
			{
				globalRotations.push_back(localRotations[childIdx]);
				globalTranslations.push_back(localTranslations[childIdx]);
			}
			else
			{
				//Rout = R1 * R2, Tout = R1 * t2 + t1
				Mat3<real> Rout; Vec3<real> Tout;
				multiplyAffineTransform4ds( globalRotations[parentIdx], globalTranslations[parentIdx], localRotations[childIdx], localTranslations[childIdx], Rout, Tout);
				globalRotations.push_back(Rout);
				globalTranslations.push_back(Tout);
			}
		}

		// set handle positions
		for (int i = 0; i < numIKJoints; ++i)
		{
			int IKJointID = IKJointIDs[i];
			handlePositions[3 * i + 0] = globalTranslations[IKJointID][0];
			handlePositions[3 * i + 1] = globalTranslations[IKJointID][1];
			handlePositions[3 * i + 2] = globalTranslations[IKJointID][2];
		}
	}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int* IKJointIDs, FK* inputFK, int adolc_tagID)
{
	this->numIKJoints = numIKJoints;
	this->IKJointIDs = IKJointIDs;
	this->fk = inputFK;
	this->adolc_tagID = adolc_tagID;

	FKInputDim = fk->getNumJoints() * 3;
	FKOutputDim = numIKJoints * 3;

	train_adolc();
}

void IK::train_adolc()
{
	// Students should implement this.
	// Here, you should setup adol_c:
	//   Define adol_c inputs and outputs. 
	//   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
	//   This will later make it possible for you to compute the gradient of this function in IK::doIK
	//   (in other words, compute the "Jacobian matrix" J).
	// See ADOLCExample.cpp .

	// Call trace_on to ask ADOL-C to begin recording how function f is implemented
	trace_on(adolc_tagID);

	// The <<= syntax tells ADOL-C that these are the input variables.
	vector<adouble> eulerAngles(FKInputDim); 
	for (int i = 0; i < FKInputDim; ++i) eulerAngles[i] <<= 0.0;

	// Computation
	vector<adouble> handlePositions(FKOutputDim);
	forwardKinematicsFunction(numIKJoints, IKJointIDs, *fk, eulerAngles, handlePositions);

	// Use >>= to tell ADOL-C that y[i] are the output variables
	vector<double> output(FKOutputDim);
	for (int i = 0; i < FKOutputDim; ++i) handlePositions[i] >>= output[i]; 

	// Call trace_off to stop recording the function f.
	trace_off(); 
}


using Eigen::MatrixXd;
using Eigen::VectorXd;
void IK::doIK(const Vec3d* targetHandlePositions, Vec3d* jointEulerAngles)
{
	// Students should implement this.
	// Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
	// Specifically, use ::function, and ::jacobian .
	// See ADOLCExample.cpp .
	//
	// Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
	// Note that at entry, "jointEulerAngles" contains the input Euler angles. 
	// Upon exit, jointEulerAngles should contain the new Euler angles.
	int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

	// Get new handle positions 
	vector<double> newHandlePositions(FKOutputDim, 0.0);
	::function(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), newHandlePositions.data());

	// Form jacobian matrix 
	vector<double> jacobianMatrix(FKOutputDim * FKInputDim);
	vector<double*> jacobianMatrixPerRow(FKOutputDim); 
	for (int r = 0; r < FKOutputDim; ++r) 
		jacobianMatrixPerRow[r] = &jacobianMatrix[r * FKInputDim];
	::jacobian(adolc_tagID, FKOutputDim, FKInputDim, jointEulerAngles->data(), jacobianMatrixPerRow.data());

	MatrixXd J(FKOutputDim, FKInputDim);
	for (int r = 0; r < FKOutputDim; ++r)
		for (int c = 0; c < FKInputDim; ++c)
			J(r, c) = jacobianMatrix[r * FKInputDim + c];

	// delta b
	VectorXd delta_b(FKOutputDim); 
	for (int i = 0; i < numIKJoints; ++i)
	{
		delta_b[3 * i + 0] = targetHandlePositions[i][0] - newHandlePositions[3 * i + 0];
		delta_b[3 * i + 1] = targetHandlePositions[i][1] - newHandlePositions[3 * i + 1];
		delta_b[3 * i + 2] = targetHandlePositions[i][2] - newHandlePositions[3 * i + 2];
	}

	// delta angle/theta
	VectorXd delta_t(FKInputDim);
	
	// compute IK
	computeIK(J, delta_b, delta_t);


	// update result
	for (int i = 0; i < numJoints; ++i)
	{
		jointEulerAngles[i][0] += delta_t[3 * i + 0]; 
		jointEulerAngles[i][1] += delta_t[3 * i + 1]; 
		jointEulerAngles[i][2] += delta_t[3 * i + 2]; 
	}
}

// See:http://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf
void IK::computeIK(Eigen::MatrixXd J, Eigen::VectorXd &delta_b, Eigen::VectorXd &delta_t)
{
	// divide IK process if move handles for a long distance
	const double moveDistanceLimit = 0.1;
	bool dividedIK = false;
	for (int i = 0; i < FKOutputDim; ++i)
		if (delta_b[i] > moveDistanceLimit) dividedIK = true;

	// if need divide
	if (dividedIK)
	{
		delta_b *= 0.5;
		computeIK(J, delta_b, delta_t);
		delta_t *= 2.0;
	}
	// else compute IK using:
	// The pseudoinverse method or
	// The Jacobian transpose method or
	// Tikhonov regularization method or
	// Damped least squares method
	else
	{
		MatrixXd J_T = J.transpose();
		MatrixXd I = MatrixXd::Identity(FKInputDim, FKInputDim);
		// for tikhonov
		double alpha1 = 0.01; 
		// for transpose
		double alpha2;
		VectorXd JJTb = J * J_T * delta_b;
		// for DLS
		double lambda = 0.01;

		switch (ikMethods)
		{
		case pseudoinverseIK:
			delta_t = J_T * (J * J_T).inverse() * delta_b;
			break;
		case tikhonovIK:
			delta_t = (J_T * J + alpha1 * I).ldlt().solve(J_T * delta_b);
			break;
		case transposeIK:
			alpha2 = delta_b.dot(JJTb) / JJTb.dot(JJTb);
			delta_t = alpha2 * J_T * delta_b;
		case dlsIK:
			delta_t = (J_T * J + lambda * lambda * I).inverse() * J_T * delta_b;
		default:
			break;
		}
	}

}
