#include "Maths.h"


void normalisePointF (PointF *vec)
{
	float len;
	len = sqrt (vec->x * vec->x + vec->y * vec->y);
	DEBUG_ASSERT (fabs (len - 1.0f) < FLT_EPSILON)
	vec->x = vec->x / len;
	vec->y = vec->y / len;
}


void multMatrices (const float *A, const float *B, float *out)
{
	out[0] = A[0] * B[0] + A[1] * B[2];
	out[1] = A[0] * B[1] + A[1] * B[3];
	out[2] = A[2] * B[0] + A[3] * B[2];
	out[3] = A[2] * B[1] + A[3] * B[3];
}

void matrixTranspose (const float *A, float *out)
{
	out[0] = A[0];
	out[1] = A[2];
	out[2] = A[1];
	out[3] = A[3];
}

void multMatricesABA (const float *A, const float *B, float *out)
{
	float temp1[4];
	float temp2[4];

	multMatrices (A, B, temp1);
	matrixTranspose (A, temp2);
	multMatrices (temp1, temp2, out);
}

PointF Maths_multMatrixByPoint (const float *A, const PointF P)
{
	PointF output;
	output.x = A[0] * P.x + A[1] * P.y;
	output.y = A[2] * P.x + A[3] * P.y;
	return output;
}

Vector4F Maths_addMatrices (const float *A, const float *B)
{
	UnionVector4F output;
	output.mat[0] = A[0] + B[0];
	output.mat[1] = A[1] + B[1];
	output.mat[2] = A[2] + B[2];
	output.mat[3] = A[3] + B[3];
	return output.vector;
}

void jacobiEigenvalues (float *cov, float *evecs)
{
	const int k = 0;
	const int m = 1;
	int i;
	float f1, f2, f3;
	float *rowk, *rowm, *vec;
	float cot2Phi, tanPhi, tanSqPhi, sinSqPhi, cosSqPhi, sinPhi, cosPhi, sin2Phi;

	// Initialise eigenvectors.
	evecs[0] = 1.0f;
	evecs[1] = 0.0f;
	evecs[2] = 0.0f;
	evecs[3] = 1.0f;

	if (fabs(cov[1]) < FLT_EPSILON)
	{
		return;
	}

	rowk = cov;
	rowm = cov + 2;

	// Calculate rotation.
	cot2Phi = 0.5f * (rowk[k] - rowm[m]) / rowk[m];
	f1 = (float)sqrt ((float)(cot2Phi * cot2Phi + 1.0f));
	if (cot2Phi < 0.0f)
	{
		f1 = -f1;
	}
	tanPhi = -cot2Phi + f1;
	tanSqPhi = tanPhi * tanPhi;
	sinSqPhi = tanSqPhi / (1.0f + tanSqPhi);
	cosSqPhi = 1.0f - sinSqPhi;
	sinPhi = (float)sqrt((float)(sinSqPhi));
	if (tanPhi < 0.0f)
	{
		sinPhi = - sinPhi;
	}
	cosPhi = (float)sqrt((float)(cosSqPhi)); 
	sin2Phi = 2.0f * sinPhi * cosPhi;

	// Rotoate covariance and eigenvector matrices.
	f1 = rowk[k];
	f2 = rowm[m];
	f3 = rowk[m];
	rowk[k] = f1 * cosSqPhi + f2 * sinSqPhi + f3 * sin2Phi;
	rowm[m] = f1 * sinSqPhi + f2 * cosSqPhi - f3 * sin2Phi;
	rowk[m] = 0.0f;
	rowm[k] = 0.0f;

	for (vec = evecs, i = 0; i < 2; vec += 2, ++i)
	{
		f1 = vec[k];
		f2 = vec[m];
		vec[k] = f1 * cosPhi + f2 * sinPhi;
		vec[m] = -f1 * sinPhi + f2 * cosPhi;
	}
}

void multMatrix (float *mat, const float k)
{
	mat[0]*=k;
	mat[1]*=k;
	mat[2]*=k;
	mat[3]*=k;
}

void addMatrices (
							const float *a1,
							const float *a2,
							float *aResult)
{
	aResult[0] = a1[0] + a2[0];
	aResult[1] = a1[1] + a2[1];
	aResult[2] = a1[2] + a2[2];
	aResult[3] = a1[3] + a2[3];
}

float fastSqrt_Bab_2 (const float x)
{
	union
	{
		int i;
		float x;
	} u;
	u.x = x;
	u.i = (1<<29) + (u.i >> 1) - (1<<22); 

	// Two Babylonian Steps (simplified from:)
	// u.x = 0.5f * (u.x + x/u.x);
	// u.x = 0.5f * (u.x + x/u.x);
	u.x =       u.x + x/u.x;
	u.x = 0.25f*u.x + x/u.x;

	return u.x;
}

void boxMuller (float *normal0, float *normal1, FILE *stream)
{
	int rand0, rand1;
	float uniform0, uniform1, s, temp;

	do
	{
		rand0 = rand();
		rand1 = rand();
#ifdef PRINT_EVENTS
		if (stream)
		{
			fprintf (stream, "<RandomVals>rand0=%d rand1=%d</RandomVals>\n", rand0, rand1);
		}
#endif
		uniform0 = (float)rand0 / RAND_MAX;
		uniform1 = (float)rand1 / RAND_MAX;
		s = uniform0 * uniform0 + uniform1 * uniform1;
	} while (s >= 1.0f || s == 0.0f || (max (uniform0, uniform1) / min (uniform0, uniform1) > 5.0f));

	// x1 = u1 * sqrt(-2.0f * log(s) / s)
	// x2 = u2 * sqrt(-2.0f * log(s) / s)

	// Note: log() is natural log (base e), while log10() is base 10
	temp = sqrt (-2.0f * log(s) / s);

	*normal0 = uniform0 * temp;
	*normal1 = uniform1 * temp;
}

int uchar_lessThan (const uchar a,
						  const void *b)
{
	uchar *u = (uchar*)b;
	return (a < *u);
}

int uchar_greaterThan (const uchar a,
						   const void *b)
{
	uchar *u = (uchar*)b;
	return (a > *u);
}

int uchar_equals (const uchar a,
					  const void *b)
{
	uchar *u = (uchar*)b;
	return (a == *u);
}


float calcMoveMotorBurst (const float dist)
{
	//const float movePerUs = MOVE_US_DELTA_FWD;
	//const float usReqdPerCell = (1.0f / movePerUs);
	//float d;

	//// Dists are, as always, in mm, but translated to simulation space => 1 cell : 20mm
	//d = (dist - BASE_MOVE_DELTA_FWD);
	//if (d > 0.0f)
	//{
	//	return d * usReqdPerCell;
	//}
	//return 0.0f;

	float d;

	// Dists are, as always, in mm, but translated to simulation space => 1 cell : 20mm
	d = (dist - BASE_MOVE_DELTA_FWD);
	if (d > 0.0f)
	{
		return d * (1.0f / MOVE_US_DELTA_FWD);
	}
	return 0.0f;
}

void calcClosestMotorDist (const float moveDist, float *motorDist, int *motorBurst)
{
	int i;
	float f, f2;

	f = (moveDist - BASE_MOVE_DELTA_FWD);
	if (0 == f)
	{
		*motorDist = moveDist;
		*motorBurst = 0;
	}

	// Calculate ms required to achieve this distance.
	f *= (1.0f / MOVE_US_DELTA_FWD);

	f2 = floor (f);
	i = (int)f2;
	if ((f - f2) >= 0.5f)
	{
		++i;
	}

	*motorDist = BASE_MOVE_DELTA_FWD + i * MOVE_US_DELTA_FWD;
	*motorBurst = i;
}




