/*!
\file RobotCore.h
\brief Commonly used operations.
*/
#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H


#include "RobotTypes.h"
#include "Image.h"


//#define TEST_SOCKETS

#if defined(BOARD) || defined(TEST_SOCKETS)
void initialiseCriticalSections();
void deleteCriticalSections();
void enterBoardData();
void leaveBoardData();
#endif // defined(BOARD) || defined(TEST_SOCKETS)

void Robot_sleep (const int milliseconds);

//! Setup folder for log files.
void setupExperimentDir();

//! Normalise the given 2D vector in-place.
void normalisePointF (PointF *vec);

//! Return transpose of given 2x2 matrix.
void matrixTranspose (const float *A, float *out);

//! Multiply matrix by scalar k.
void multMatrix (float *mat, const float k);

//! Multiply two 2x2 matrices.
void multMatrices (const float *A, const float *B, float *out);

//! Given 2x2 matrices A and B, return ABA
/*!
\verbatim
a b x d e = ad+bf ae+bg
c d   f g   cd+df ce+dg
\endverbatim
*/
void multMatricesABA (const float *A, const float *B, float *out);

//! Calculate the eigenvalues and eigenvectors of the given symmetrix 2x2 matrix.
/*!
The given matrix is altered and upon return should contain the eigenvalues in the
diagonal components. The approach is based on the Jacobi cyclic method, but much
simpler as only one rotation is required. mymathlib.com was used as a reference.
*/
void jacobiEigenvalues (float *cov, float *eigenvectors);

//! Add two 2x2 matrices
void addMatrices (const float *a1, const float *a2, float *aResult);

int RobotCore_checkLine (
	const int beginX,
	const int beginY,
	const int endX,
	const int endY,
	const Image *img,
	int (*condition)(const uchar a, const void *b),
	const int searchValue,
	const BRESENHAM_TYPE type,
	const int isDistReqd,
	uchar *hitValue);

int RobotCore_checkFile (
	const schar *fileName,
	const schar *delimeter);


int RobotCore_comparePrevRobotPose (
	const Pose *currPose,
	const PointI *prevLoc,
	const int prevOrient);

int RobotCore_copyImage_specColour (
	Image *src,
	Image *dest,
	const uchar specColour);

//! From http://ilab.usc.edu/wiki/index.php/Fast_Square_Root
float fastSqrt_Bab_2 (const float x);

//! Generate a Gaussian distributed random number
/*!
We're actually using the polar form of Box-Muller here, or the Marsiglia
polar method. This method avoids the trigonometric functions required by
Box-Muller, but required a while loop as points outside of the unit circle
are rejected.

The rand function is seeded in the Robot constructor.
*/
void boxMuller (float *normal0, float *normal1, FILE *stream);

//! Return 1 if a is less than b
int uchar_lessThan (const uchar a,
						  const void *b);

//! Return 1 if a is greater than b
int uchar_greaterThan (const uchar a,
						   const void *b);

//! Returns 1 if a equals b
int uchar_equals (const uchar a,
					  const void *b);

//! Returns 1 if the uchar is in (actually occupied, narrow occupied, narrow vehicle)
int RobotCore_equalsNarrowOccupied (const uchar a, const void *b);

//! Returns 1 if the uchar is in (actually occupied, narrow vehicle), i.e. is there visual contact.
int RobotCore_equalsActualOccupied (const uchar a, const void *b);

int estMapScanGain (const PointF optimumPt,
					const PointI localMapOrigin,
					__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS]);

void RobotCore_printPointFList (List *pointIList, FILE *f);

//! Find coalition in list in which the robot of given index is an explorer.
Coalition findExplorationCoalition (List *coalitions, const int index);

//! Copy coalitions in which the robot of given index is a supervisor
void findSupervisionCoalitions (List *boardCoalitions, List *robotCoalitions, const int index);


#endif // ifndef


