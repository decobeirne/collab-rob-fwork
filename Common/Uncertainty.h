/*!
\file Uncertainty.h
\brief Robot positional certainty.
*/
#ifndef UNCERTAINTY_H
#define UNCERTAINTY_H

#include "Point.h"
#include "Vector.h"
#include "BaseTypes.h"
#include "RobotTypes.h"

void Uncertainty_scaleCovToSim (float *cov);

//! Return type is just Vector4F for simplicity's sake
Vector4F Uncertainty_calcIndividualCovariance (
	const float *cov,
	const float orient);

void Uncertainty_calcEigenvalues (const float *cov, float *evals, PointF *evec, float *stdDev);

//! Update the robot's covariance matrix given the covariance matrix for a move and the orient of the move
/*!
The Jacobian matrix that translates a vector from the move space to the robot's location space is
calculated, and the location covariance is updated using this
*/
void Uncertainty_updateCovariance (
	const float *covMove,
	const float orient,
	float *cov);

#ifdef SIM_ERROR
//! Generate error in robot's location and add to accumulated error.
/*!
Calculate random numbers based on eigenvalues of covariance representing second
moments of probability distribution. Translate error from eigenvectors back to
robot coordinates.

Given uniformly distributed random variables in the interval 0 to 1, the Box-Muller
method used will return two normally distributed random variables with mean 0 and
variance of 1.
*/
PointF Uncertainty_accumulateError (
	const float eval0,
	const float eval1,
	const PointF evec,
	PointF *accumulatedError,
	FILE *fstream,
	const int forMove);
#endif // ifdef SIM_ERROR

//! Update robot's covariance based on last move carried out
/*!
The last move made by the robot is used to update the covariance matrix representing
it's location estimate. The location estimate is maintained as an approximation of the
probability over [x,y]^T. The first and second moments of this distribution are
represented as the mean and covariance respectively.
*/
Vector4F Uncertainty_updateCovarianceForMove (
	const Move *m,
	const UncertaintyConstants *u,
	Pose *p,
	float *stdDev,
	FILE *stream);

#ifdef SIM_ERROR
void Uncertainty_accumErrorForMove (const float covMove[4], PointF *actualLocOffset, FILE *fstream);
#endif // ifdef SIM_ERROR

//! Calculate covariance of error for single estimate of location of visible robot face
Vector4F Uncertainty_calcVisualEstimateCovariance (
	const UncertaintyConstants *u,
	const float robotOrient,
	const float varFwd,
	const float varLat,
	const float covar);



//! Given cov for projecting point from image to world, accum error for position of point along line
/*!
Used in RobotRecognitionRoughEstimation, the initial error is that related to projecting a pixel
in the robot's camera image to world space. This is a point on the top/bottom edge of the visible
face. In some instances, we will not be able to determine where this point is along the edge of
the face, e.g. it could be anywhere along the top edge of the face. The v coarse estimation is
made that these points are uniformly distributed across the length of the edge. We can therefore
calc a covariance for this error that is oriented along the face orientation. This is accumulated
onto the initial cov to give the final error covariance matrix.
*/
Vector4F Uncertainty_accumRobotFaceHorizonalEdgeEstimateCovariance (
	const UncertaintyConstants *u,
	const Vector4F initialCov,
	const float faceOrient,
	const float faceHalfLen);

//! Calculate error accumulated due to estimate of location from visual data.
Vector4F Uncertainty_updateCovarianceForVisualEst (
	const UncertaintyConstants *uncertaintyConstants,
	const float robotDist,
	const float orient,
	Pose *pose,
	float *stdDev);

#ifdef SIM_ERROR
PointF Uncertainty_accumErrorForVisualEst (
	const float covVisualEst[4],
	PointF *actualLocOffset,
	FILE *fstream);
#endif


#endif // ifndef UNCERTAINTY_H


