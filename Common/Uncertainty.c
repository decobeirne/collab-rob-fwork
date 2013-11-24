
#include "Uncertainty.h"
#include "Geometry.h"
#include "Maths.h"
#include "RobotTypes.h"

extern float ATAN_ORIENT_VAR;

void Uncertainty_updateCovariance (const float *covMove,
								   const float orient,
								   float *cov)
{
	float jacMove[4];
	float temp[4];
	float latOrient;

	latOrient = Geometry_orientSum (orient, PI/2.0f);

	// Calculate the Jacobian matrix to translate from the movement space to the
	// robot's location space.
	//
	// x = fwd * cos(fwdOrient) + lat * cos(latOrient)
	// y = fwd * sin(fwdOrient) + lat * sin(latOrient)
	//
	// Jacobian is therefore: dx/df dx/dl dy/df dy/dl
	jacMove[0] = (float)cos(orient);
	jacMove[1] = (float)cos(latOrient);
	jacMove[2] = (float)sin(orient);
	jacMove[3] = (float)sin(latOrient);

	// Multiply Jacobian to get new covariance matrix
	//
	// covNew = I * covOld * I + jacMove * covMove * jacMove^T
	multMatricesABA (jacMove, covMove, temp);
	addMatrices (cov, temp, cov);
}

void Uncertainty_scaleCovToSim (float *cov)
{
	const float f = (1.0f / (MAP_SCALE * MAP_SCALE));
	cov[0] *= f;
	cov[1] *= f;
	cov[2] *= f;
	cov[3] *= f;
}

//! Return type is just Vector4F for simplicity's sake
Vector4F Uncertainty_calcIndividualCovariance (
	const float *cov,
	const float orient)
{
	float jacMove[4];
	UnionVector4F newCov;
	float latOrient;

	latOrient = Geometry_orientSum (orient, RADS_90);

	// Calculate the Jacobian matrix to translate from the movement space to the
	// robot's location space.
	//
	// x = fwd * cos(fwdOrient) + lat * cos(latOrient)
	// y = fwd * sin(fwdOrient) + lat * sin(latOrient)
	//
	// Jacobian is therefore: dx/df dx/dl dy/df dy/dl
	jacMove[0] = (float)cos(orient);
	jacMove[1] = (float)cos(latOrient);
	jacMove[2] = (float)sin(orient);
	jacMove[3] = (float)sin(latOrient);

	// Multiply Jacobian to get new covariance matrix
	//
	// covNew = I * covOld * I + jacMove * covMove * jacMove^T
	multMatricesABA (jacMove, cov, newCov.mat);

	return newCov.vector;
}

void Uncertainty_calcEigenvalues (const float *cov, float *evals, PointF *evec, float *stdDev)
{
	float temp[4];
	float evecs[4];

	// Calculate the eigenvalues and eigenvectors of the error ellipse
	// represented by the robot's location covariance matrix. The square
	// root of the eigenvalues will represent the standard deviation of
	// the robot's location estimate along the corresponding eigenvector.
	memcpy (temp, cov, sizeof (float) * 4);
	jacobiEigenvalues (temp, evecs);
	DEBUG_ASSERT(temp[0] >= 0.0f)
	DEBUG_ASSERT(temp[3] >= 0.0f)
	evals[0] = sqrt(temp[0]);
	evals[1] = sqrt(temp[3]);
	evec->x = evecs[0];
	evec->y = evecs[1];
	normalisePointF (evec);

	// Calculate the area of the error ellipse. This is used when determining
	// the profit of exploring terrain.
	*stdDev = evals[0] * evals[1];
}

int accumTemp = 0;
int accumTemp2 = 0;
#ifdef SIM_ERROR
PointF Uncertainty_accumulateError (
	const float eval0,
	const float eval1,
	const PointF evec0,
	PointF *accumulatedError,
	FILE *fstream,
	const int forMove)
{
	PointF simdError;
	float normal0, normal1;
//	float scaledNormal0, scaledNormal1;
	PointF evec1;

	boxMuller (&normal0, &normal1, fstream);

#if 0
	normal0 *= 2.0f; normal0 -= 1.0f;
	normal1 *= 2.0f; normal1 -= 1.0f;
#endif

#if 1
	// Temp - playing with odometry errors
	if (accumTemp)
	{
		normal0 = -normal0;
		normal1 = -normal1;
	}
	accumTemp = !accumTemp;
#endif

#if 1
	if (forMove)
	{
#if 1
		normal0 *= 3; // 3 standard deviations will capture 99.7% of a normal distribution
		normal1 *= 3;
#endif

#if 1
		normal0 *= 2; // Testing
		normal1 *= 2;
#endif
	}
#endif

#if 0
	if (accumTemp2 % 5 == 0)
	{
		normal0 *= 5;
		normal1 *= 5;

		accumTemp2 = 0;
	}
#endif

	normal0 *= eval0;
	normal1 *= eval1;

	evec1.x = -evec0.y;
	evec1.y = evec0.x;

	simdError.x = normal0 * evec0.x + normal1 * evec1.x;
	simdError.y = normal0 * evec0.y + normal1 * evec1.y;

	if (fstream)
	{
		// Sanity check errors if we are recording or rerunning
#if defined (PRINT_EVENTS) || defined (RECORDING_ROBOT) || defined (RERUNNING_ROBOT)
		fprintf (fstream, "<SimulateError>evec0=(%f,%f) evec1=(%f,%f) eval0=%f eval1=%f normals=(%f,%f) error=(%f,%f)</SimulateError>\n",
			evec0.x, evec0.y, evec1.x, evec1.y, eval0, eval1, normal0, normal1, simdError.x, simdError.y);
#endif
	}

	accumulatedError->x += simdError.x;
	accumulatedError->y += simdError.y;

	return simdError;
}
#endif // ifdef SIM_ERROR

Vector4F Uncertainty_updateCovarianceForMove (
	const Move *m,
	const UncertaintyConstants *u,
	Pose *p,
	float *stdDev,
	FILE *stream)
{
	UnionVector4F cov;

	// Calculate covariance matrix for the given move.
	//
	// Variance is measured in the direction of the robot's initial orientation,
	// fwdOrient, and perpendicular to this, latOrient.
	switch (m->dir)
	{
	case 0: // Fwd
		cov.mat[0] = BASE_MOVE_VAR_FWD + m->usBurst * MOVE_US_VAR_FWD;
		cov.mat[1] = BASE_MOVE_COVAR + m->usBurst * MOVE_US_COVAR;
		cov.mat[2] = cov.mat[1];
		cov.mat[3] = BASE_MOVE_VAR_LAT + m->usBurst * MOVE_US_VAR_LAT;

		// Also consider uncertainty in robot's orient.
		EXPLICIT_DEBUG_ASSERT (u->isSetup == 1)
//		temp = (BASE_MOVE_DELTA_FWD + m->usBurst * MOVE_US_DELTA_FWD);
//		temp = u->atanOrientVar * temp;
//		cov.mat[3] += temp;
		cov.mat[3] += ATAN_ORIENT_VAR * (BASE_MOVE_DELTA_FWD + m->usBurst * MOVE_US_DELTA_FWD);
		break;
	case 2: // Left
		cov.mat[0] = ROT_LEFT_VAR_FWD;
		cov.mat[1] = ROT_LEFT_COVAR;
		cov.mat[2] = ROT_LEFT_COVAR;
		cov.mat[3] = ROT_LEFT_VAR_LAT;
		break;
	case 3: // Right
		cov.mat[0] = ROT_RIGHT_VAR_FWD;
		cov.mat[1] = ROT_RIGHT_COVAR;
		cov.mat[2] = ROT_RIGHT_COVAR;
		cov.mat[3] = ROT_RIGHT_VAR_LAT;
		break;
	}

#if 1
	if (1)
	{
		cov.mat[0] *= 2; // Testing
		cov.mat[1] *= 2;
		cov.mat[2] *= 2;
		cov.mat[3] *= 2;
	}
#endif

	Uncertainty_updateCovariance (cov.mat, p->orient, p->mat);
	Uncertainty_calcEigenvalues (p->mat, p->evals, &p->evec, stdDev);
#if 1
	if (stream)
	{
		fprintf (stream, "<UpdateCovForMove>matIn=(%f,%f,%f,%f) matOut=(%f,%f,%f,%f) orient=%f evec=(%f,%f) evals=(%f,%f)</UpdateCovForMove>\n",
			cov.mat[0], cov.mat[1], cov.mat[2], cov.mat[3],
			p->mat[0], p->mat[1], p->mat[2], p->mat[3],
			p->orient,
			p->evec.x, p->evec.y,
			p->evals[0], p->evals[1]);
	}
#endif
	return cov.vector;
}

#ifdef SIM_ERROR
void Uncertainty_accumErrorForMove (const float covMove[4], PointF *actualLocOffset, FILE *fstream)
{
	float evals[2];
	PointF evec;
	float temp;

	// The eigenvalues are the variance in the robot's location estimate
	// distibution along the principal axes of the distribution.
	Uncertainty_calcEigenvalues ((const float*)covMove, evals, &evec, &temp);
	Uncertainty_accumulateError (
		evals[0],
		evals[1],
		evec,
		actualLocOffset,
		fstream,
		1);
}
#endif // ifdef SIM_ERROR

Vector4F Uncertainty_calcVisualEstimateCovariance (
	const UncertaintyConstants *u,
	const float robotOrient,
	const float varFwd,
	const float varLat,
	const float covar)
{
	UnionVector4F cov;

	// Un-rotated covariance for coarse location estimate for visible robot face
	cov.mat[0] = varFwd;
	cov.mat[1] = covar;
	cov.mat[2] = covar;
	cov.mat[3] = varLat;

	// Account for (small) uncertainty in robot's orient
	cov.mat[3] += ATAN_ORIENT_VAR * u->visEst_typicalDist;

	cov.vector = Uncertainty_calcIndividualCovariance (cov.mat, robotOrient);

	return cov.vector;
}

Vector4F Uncertainty_accumRobotFaceHorizonalEdgeEstimateCovariance (
	const UncertaintyConstants *u,
	const Vector4F initialCov,
	const float faceOrient,
	const float faceHalfLen)
{
	UnionVector4F initialUnion;
	UnionVector4F estUnion;
	float var;

	initialUnion.vector = initialCov;

	var = (faceHalfLen * 0.5f) * (faceHalfLen * 0.5f);
	estUnion.mat[0] = 0.0f;
	estUnion.mat[1] = 0.0f;
	estUnion.mat[2] = 0.0f;
	estUnion.mat[3] = var;

	Uncertainty_updateCovariance (estUnion.mat, faceOrient, initialUnion.mat);

	return initialUnion.vector;
}

Vector4F Uncertainty_updateCovarianceForVisualEst (
	const UncertaintyConstants *u,
	const float robotDist,
	const float orient,
	Pose *pose,
	float *stdDev)
{
	UnionVector4F cov;

	// Calculate covariance matrix for the location estimate.
	cov.mat[0] = u->visEst_final_varFwd;
	cov.mat[1] = u->visEst_final_covar;
	cov.mat[2] = u->visEst_final_covar;
	cov.mat[3] = u->visEst_final_varLat;

	// Account for (small) uncertainty in robot's orient
	EXPLICIT_DEBUG_ASSERT (u->isSetup == 1)
	cov.mat[3] += ATAN_ORIENT_VAR * robotDist;

	Uncertainty_scaleCovToSim (cov.mat);
	Uncertainty_updateCovariance (cov.mat, orient, pose->mat);
	Uncertainty_calcEigenvalues (pose->mat, pose->evals, &pose->evec, stdDev);

	return cov.vector;
}

#ifdef SIM_ERROR
PointF Uncertainty_accumErrorForVisualEst (
	const float covVisualEst[4],
	PointF *actualLocOffset,
	FILE *fstream)
{
	float evals[2];
	PointF evec;
	float temp;

	// The eigenvalues are the variance in the robot's location estimate
	// distibution along the principal axes of the distribution.
	Uncertainty_calcEigenvalues (covVisualEst, evals, &evec, &temp);
	return Uncertainty_accumulateError (
		evals[0],
		evals[1],
		evec,
		actualLocOffset,
		fstream,
		0);
}
#endif // SIM_ERROR






