#include "ObstacleRecognitionCore.h"

#ifndef BOARD

//#define ASDF
#ifdef ASDF

class CellDestruction
{
	


};

CellDescription ObstacleRecognition::trainCellDescriptio(
	const Image *img,
	const int index)
{
	CellDescription cellDescription;
	// Calculate 
	return cellDescription;
}


ImageDescription ObstacleRecognition::trainImageDescription(
	const Image *image)
{
  CellDescription cellDescriptions[N_CELLS_IN_IMG];
  for (int i = 0; i < N_CELLS_IN_IMG; ++i)
  {
    cellDescriptions[i] = trainCellDescription(image, i);
  }
  ImageDescription imageDescription = trainI


#endif // ASDF




int ObstacleRecognitionCore_cellXToPixelX (
	const int coord)
{
	return (coord * CAM_OCCUPANCY_CELL_X) + CAM_OCCUPANCY_CELL_X_HALF;
}

int ObstacleRecognitionCore_cellYToPixelY (
	const int coord)
{
	return (coord * CAM_OCCUPANCY_CELL_Y) + CAM_OCCUPANCY_CELL_Y_HALF;
}

PointI ObstacleRecognitionCore_cellCoordsToPixelCoords (
	const PointI coords)
{
	PointI pixels;
	pixels.x = (coords.x * CAM_OCCUPANCY_CELL_X) + CAM_OCCUPANCY_CELL_X_HALF;
	pixels.y = (coords.y * CAM_OCCUPANCY_CELL_Y) + CAM_OCCUPANCY_CELL_Y_HALF;
	return pixels;
}

void ObstacleRecognitionCore_calcImgFeatures (
	ImgFeatures *imgFeatures,
	ImgCellFeatures *cellFeatures,
	const int occupancyGridY)
{
	int i, j;
	const float nCellsInv = 1.0f / (CAM_OCCUPANCY_GRID_X * occupancyGridY);

	for (i = 0; i < 12; ++i)
	{
		imgFeatures->vals[i] = 0.0f;
	}
imgFeatures->nCellsWithWeirdPixels = 0;
	for (j = 0; j < occupancyGridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			imgFeatures->vals[0] += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[0];
			imgFeatures->vals[1] += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[1];
			imgFeatures->vals[2] += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[2];
			imgFeatures->vals[3] += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[3];
			imgFeatures->vals[4] += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[4];
			imgFeatures->vals[5] += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[5];
			imgFeatures->nCellsWithWeirdPixels += cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].hasWeirdPixels;
		}
	}
//printf("nCellsWithWeirdPixels %d\n", imgFeatures->nCellsWithWeirdPixels);
	imgFeatures->vals[0] *= nCellsInv;
	imgFeatures->vals[1] *= nCellsInv;
	imgFeatures->vals[2] *= nCellsInv;
	imgFeatures->vals[3] *= nCellsInv;
	imgFeatures->vals[4] *= nCellsInv;
	imgFeatures->vals[5] *= nCellsInv;

	for (j = 0; j < occupancyGridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			imgFeatures->vals[6] += fabs (cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[0] - imgFeatures->vals[0]);
			imgFeatures->vals[7] += fabs (cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[1] - imgFeatures->vals[1]);
			imgFeatures->vals[8] += fabs (cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[2] - imgFeatures->vals[2]);
			imgFeatures->vals[9] += fabs (cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[3] - imgFeatures->vals[3]);
			imgFeatures->vals[10] += fabs (cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[4] - imgFeatures->vals[4]);
			imgFeatures->vals[11] += fabs (cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[5] - imgFeatures->vals[5]);
		}
	}

	imgFeatures->vals[6] *= nCellsInv;
	imgFeatures->vals[7] *= nCellsInv;
	imgFeatures->vals[8] *= nCellsInv;
	imgFeatures->vals[9] *= nCellsInv;
	imgFeatures->vals[10] *= nCellsInv;
	imgFeatures->vals[11] *= nCellsInv;
}

void ObstacleRecognitionCore_calcInvImgFeatures (
	ImgFeatures *imgFeatures,
	ImgFeatures *invImgFeatures)
{
	int i;
	for (i = 0; i < 12; ++i)
	{
		invImgFeatures->vals[i] = 1.0f / imgFeatures->vals[i];
	}
}

void ObstacleRecognitionCore_calcCellFeaturesOLD (
	Image *img,
	const int i,
	const int j,
	ImgCellFeatures *cellFeatures,
	const int gridOrigX,
	const int gridOrigY)
{
	int k, l;
	int yStart, yEnd;
	int xStart, xEnd;
	uchar *imgPtr;
	uchar *imgPtr2;
	const float nInv = 1.0f / (CAM_OCCUPANCY_CELL_X * CAM_OCCUPANCY_CELL_Y);
	const float n2Inv = 1.0f / ((CAM_OCCUPANCY_CELL_X - 1) * (CAM_OCCUPANCY_CELL_Y - 1));

	float hAvg;
	float sAvg;
	float vAvg;
	float hTextAvg;
	float sTextAvg;
	float vTextAvg;
	int hDiff, sDiff, vDiff;

	hAvg = 0.0f;
	sAvg = 0.0f;
	vAvg = 0.0f;
	hTextAvg = 0.0f;
	sTextAvg = 0.0f;
	vTextAvg = 0.0f;

	yStart = gridOrigY + j * CAM_OCCUPANCY_CELL_Y;
	yEnd = yStart + CAM_OCCUPANCY_CELL_Y;

	for (l = yStart; l < yEnd; ++l)
	{
		imgPtr = img->data + (l * img->wStep) + (gridOrigX * 3) + (i * CAM_OCCUPANCY_CELL_X * 3);

		for (k = 0; k < CAM_OCCUPANCY_CELL_X; ++k, imgPtr+=3)
		{
			hAvg += imgPtr[0];
			sAvg += imgPtr[1];
			vAvg += imgPtr[2];
		}
	}

	--yEnd;
	for (l = yStart; l < yEnd; ++l)
	{
		xStart = gridOrigX + i * CAM_OCCUPANCY_CELL_X;
		xEnd = xStart + CAM_OCCUPANCY_CELL_X;
		--xEnd;

		for (k = xStart; k < xEnd; ++k)
		{
			hDiff = 0;
			sDiff = 0;
			vDiff = 0;
			imgPtr = img->data + (l * img->wStep + k * 3);
			imgPtr2 = img->data + ((l    ) * img->wStep + (k + 1) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			imgPtr2 = img->data + ((l + 1) * img->wStep + (k    ) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			imgPtr2 = img->data + ((l + 1) * img->wStep + (k + 1) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			hTextAvg += (float)hDiff;
			sTextAvg += (float)sDiff;
			vTextAvg += (float)vDiff;
		}
	}

	hAvg *= nInv;
	sAvg *= nInv;
	vAvg *= nInv;
	hTextAvg *= n2Inv;
	sTextAvg *= n2Inv;
	vTextAvg *= n2Inv;

	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[0] = hAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[1] = sAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[2] = vAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[3] = hTextAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[4] = sTextAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[5] = vTextAvg;
}

#define KERNEL_DIMS 5
#define KERNEL_DIMS_MINUS_1 4
ImgCellFeatures ObstacleRecognitionCore_calcKernelFeatures (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	Image *img,
	const PointI kernelCentre)
{
	PointI bottomCorner;
	int i, j;
	uchar *imgPtr, *imgPtr2;
	ImgCellFeatures cellFeatures;

	float hAvg;
	float sAvg;
	float vAvg;
	float hTextAvg;
	float sTextAvg;
	float vTextAvg;
	int hDiff, sDiff, vDiff;

	const float nInv = 1.0f / (KERNEL_DIMS * KERNEL_DIMS);
	const float n2Inv = 1.0f / ((KERNEL_DIMS_MINUS_1) * (KERNEL_DIMS_MINUS_1));

	hAvg = 0.0f;
	sAvg = 0.0f;
	vAvg = 0.0f;
	hTextAvg = 0.0f;
	sTextAvg = 0.0f;
	vTextAvg = 0.0f;

	// Assume kernel centre pt is ok
	bottomCorner.x = kernelCentre.x - 2;
	bottomCorner.y = kernelCentre.y - 2;

	for (j = bottomCorner.y; j < (bottomCorner.y + KERNEL_DIMS); ++j)
	{
		imgPtr = img->data + (j * img->wStep) + (bottomCorner.x * 3);

		for (i = bottomCorner.x; i < (bottomCorner.x + KERNEL_DIMS); ++i, imgPtr+=3)
		{
			hAvg += imgPtr[0];
			sAvg += imgPtr[1];
			vAvg += imgPtr[2];
		}
	}

	for (j = bottomCorner.y; j < (bottomCorner.y + KERNEL_DIMS_MINUS_1); ++j)
	{
		imgPtr = img->data + (j * img->wStep) + (bottomCorner.x * 3);

		for (i = bottomCorner.x; i < (bottomCorner.x + KERNEL_DIMS_MINUS_1); ++i)
		{
			hDiff = 0;
			sDiff = 0;
			vDiff = 0;
			imgPtr = img->data + (j * img->wStep + i * 3);
			imgPtr2 = img->data + ((j    ) * img->wStep + (i + 1) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			imgPtr2 = img->data + ((j + 1) * img->wStep + (i    ) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			imgPtr2 = img->data + ((j + 1) * img->wStep + (i + 1) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			hTextAvg += (float)hDiff;
			sTextAvg += (float)sDiff;
			vTextAvg += (float)vDiff;
		}
	}

	hAvg *= nInv;
	sAvg *= nInv;
	vAvg *= nInv;
	hTextAvg *= n2Inv;
	sTextAvg *= n2Inv;
	vTextAvg *= n2Inv;

	cellFeatures.vals[0] = hAvg;
	cellFeatures.vals[1] = sAvg;
	cellFeatures.vals[2] = vAvg;
	cellFeatures.vals[3] = hTextAvg;
	cellFeatures.vals[4] = sTextAvg;
	cellFeatures.vals[5] = vTextAvg;

#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "<KernelFeatures>pt=(%d,%d) features=(%f,%f,%f,%f,%f,%f)</KernelFeatures>\n",
		kernelCentre.x, kernelCentre.y, hAvg, sAvg, vAvg, hTextAvg, sTextAvg, vTextAvg);
#endif

	return cellFeatures;
}
#undef KERNEL_DIMS
#undef KERNEL_DIMS_MINUS_1


void ObstacleRecognitionCore_calcCellFeatures (
	Image *img,
	const int i,
	const int j,
	ImgCellFeatures *cellFeatures,
	const int gridOrigX,
	const int gridOrigY)
{
	int k, l;
	int yStart, yEnd;
	int xStart, xEnd;
	uchar *imgPtr;
	uchar *imgPtr2;
	const float nInv = 1.0f / (CAM_OCCUPANCY_CELL_X * CAM_OCCUPANCY_CELL_Y);
	const float n2Inv = 1.0f / ((CAM_OCCUPANCY_CELL_X - 1) * (CAM_OCCUPANCY_CELL_Y - 1));

	float hAvg;
	float sAvg;
	float vAvg;
	float hTextAvg;
	float sTextAvg;
	float vTextAvg;
	int hDiff, sDiff, vDiff;
	int maxs[3];


	hAvg = 0.0f;
	sAvg = 0.0f;
	vAvg = 0.0f;
	hTextAvg = 0.0f;
	sTextAvg = 0.0f;
	vTextAvg = 0.0f;

	yStart = gridOrigY + j * CAM_OCCUPANCY_CELL_Y;
	yEnd = yStart + CAM_OCCUPANCY_CELL_Y;

	for (l = yStart; l < yEnd; ++l)
	{
		imgPtr = img->data + (l * img->wStep) + (gridOrigX * 3) + (i * CAM_OCCUPANCY_CELL_X * 3);

		for (k = 0; k < CAM_OCCUPANCY_CELL_X; ++k, imgPtr+=3)
		{
			hAvg += imgPtr[0];
			sAvg += imgPtr[1];
			vAvg += imgPtr[2];
		}
	}
maxs[0] = maxs[1] = maxs[2] = 0;
	--yEnd;
	for (l = yStart; l < yEnd; ++l)
	{
		xStart = gridOrigX + i * CAM_OCCUPANCY_CELL_X;
		xEnd = xStart + CAM_OCCUPANCY_CELL_X;
		--xEnd;

		for (k = xStart; k < xEnd; ++k)
		{
			hDiff = 0;
			sDiff = 0;
			vDiff = 0;
			imgPtr = img->data + (l * img->wStep + k * 3);
			imgPtr2 = img->data + ((l    ) * img->wStep + (k + 1) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			imgPtr2 = img->data + ((l + 1) * img->wStep + (k    ) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			imgPtr2 = img->data + ((l + 1) * img->wStep + (k + 1) * 3); hDiff += abs (imgPtr[0] - imgPtr2[0]); sDiff += abs (imgPtr[1] - imgPtr2[1]); vDiff += abs (imgPtr[2] - imgPtr2[2]);
			hTextAvg += (float)hDiff;
			sTextAvg += (float)sDiff;
			vTextAvg += (float)vDiff;
			maxs[0] = max(hDiff, maxs[0]);
			maxs[1] = max(sDiff, maxs[1]);
			maxs[2] = max(vDiff, maxs[2]);
//			printf("%3d %3d %3d\n", hDiff, sDiff, vDiff);
		}
	}
//	printf("max diffs ");
//	printf("%d %d %d\n", maxs[0], maxs[1], maxs[2]);
//	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].hasWeirdPixels = (maxs[0] > 150 || maxs[1] > 150 || maxs[2] > 150);
//	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].hasWeirdPixels = (maxs[0] > 100 || maxs[1] > 100 || maxs[2] > 100);
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].hasWeirdPixels = (maxs[0] > 100 && maxs[1] > 100 && maxs[2] > 100);
//	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].hasWeirdPixels = (maxs[0] > 125 && maxs[1] > 125 && maxs[2] > 125);
//	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].hasWeirdPixels = (maxs[0] > 150 && maxs[1] > 150 && maxs[2] > 150);

	hAvg *= nInv;
	sAvg *= nInv;
	vAvg *= nInv;
	hTextAvg *= n2Inv;
	sTextAvg *= n2Inv;
	vTextAvg *= n2Inv;

	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[0] = hAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[1] = sAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[2] = vAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[3] = hTextAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[4] = sTextAvg;
	cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[5] = vTextAvg;
}

void ObstacleRecognitionCore_calcImgCellFeatures (
	Image *img,
	ImgCellFeatures *cellFeatures,
	const int occupancyGridY,
	const int gridOrigX,
	const int gridOrigY)
{
	int i, j;

	for (j = 0; j < occupancyGridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
//			ObstacleRecognitionCore_calcCellFeaturesOLD (
			ObstacleRecognitionCore_calcCellFeatures (
				img,
				i,
				j,
				cellFeatures,
				gridOrigX,
				gridOrigY);
		}
	}
}

void ObstacleRecognitionCore_calcCellDescRelToImg (
	ImgCellFeatures *cellFeatures,
	ImgFeatures *imgInvFeatures,
	const int imgGroupRefFeatures[6],
	float relFeatures[6])
{
	int i;
	for (i = 0; i < 6; ++i)
	{
		relFeatures[i] = cellFeatures->vals[i] * imgInvFeatures->vals[imgGroupRefFeatures[i]];
	}
}

float ObstacleRecognitionCore_calcCellDiff (
	const float *cellDesc,
	const float *cellGroupDesc)
{
	float diff =
		fabs (cellDesc[0] - cellGroupDesc[0]) +
		fabs (cellDesc[1] - cellGroupDesc[1]) +
		fabs (cellDesc[2] - cellGroupDesc[2]) +
		fabs (cellDesc[3] - cellGroupDesc[3]) +
		fabs (cellDesc[4] - cellGroupDesc[4]) +
		fabs (cellDesc[5] - cellGroupDesc[5]);
	return diff;
}

float ObstacleRecognitionCore_calcImgDiff (
	ImgFeatures *imgFeatures,
	const float *imgGroupDesc,
	const float *imgStdDevs)
{
	float diff = 
		fabs (imgFeatures->vals[0] - imgGroupDesc[0]) / imgStdDevs[0] +
		fabs (imgFeatures->vals[1] - imgGroupDesc[1]) / imgStdDevs[1] +
		fabs (imgFeatures->vals[2] - imgGroupDesc[2]) / imgStdDevs[2] +
		fabs (imgFeatures->vals[3] - imgGroupDesc[3]) / imgStdDevs[3] +
		fabs (imgFeatures->vals[4] - imgGroupDesc[4]) / imgStdDevs[4] +
		fabs (imgFeatures->vals[5] - imgGroupDesc[5]) / imgStdDevs[5] +
		fabs (imgFeatures->vals[6] - imgGroupDesc[6]) / imgStdDevs[6] +
		fabs (imgFeatures->vals[7] - imgGroupDesc[7]) / imgStdDevs[7] +
		fabs (imgFeatures->vals[8] - imgGroupDesc[8]) / imgStdDevs[8] +
		fabs (imgFeatures->vals[9] - imgGroupDesc[9]) / imgStdDevs[9] +
		fabs (imgFeatures->vals[10] - imgGroupDesc[10]) / imgStdDevs[10] +
		fabs (imgFeatures->vals[11] - imgGroupDesc[11]) / imgStdDevs[11];
	return diff;
}

int ObstacleRecognitionCore_isImageOk (
	Image *camImg,
	ImgCellFeatures *cellFeatures,
	ImgFeatures *imgFeatures)
{
	// Some images have a load of garbage pixels in them
	if (imgFeatures->nCellsWithWeirdPixels > 100)
	{
		return 0;
	}

	// Blank images - texture should be 0
	if (imgFeatures->vals[3] < 1.0f && imgFeatures->vals[4] < 1.0f && imgFeatures->vals[5] < 1.0f)
	{
		return 0;
	}
/*	int i;
	float avgs[3];
	avgs[0] = avgs[1] = avgs[2] = 0.0f;
	for (i = 0; i < CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y; ++i)
	{
		printf ("%f,%f,%f\n", cellFeatures[i].vals[3], cellFeatures[i].vals[4], cellFeatures[i].vals[5]);
		avgs[0] += cellFeatures[i].vals[3];
		avgs[1] += cellFeatures[i].vals[4];
		avgs[2] += cellFeatures[i].vals[5];
	}
	printf ("avgs\n");
	avgs[0] /= (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	avgs[1] /= (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	avgs[2] /= (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y);
	printf ("%f,%f,%f\n", avgs[0], avgs[1], avgs[2]);*/

	return 1;
}


#endif

