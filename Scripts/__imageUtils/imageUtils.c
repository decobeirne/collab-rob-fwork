#include "Python.h"

//
// Types
//
typedef struct _CellFeatDesc
{
	float ratios[12];
} CellFeatDesc;

/*typedef struct _CellDesc
{
	CellFeatDesc feats[6];
} CellDesc;*/
typedef struct _CellDesc
{
	float feats[6];
} CellDesc;

typedef struct _RefFeats
{
	int indices[6];
} RefFeats;

typedef struct _GroupDesc
{
	float feats[6];
	int nCells;
} GroupDesc;

typedef struct _CellDiff
{
	float diff;
	unsigned short a;
	unsigned short b;
} CellDiff;

typedef CellDiff GroupDiff;

#define N_GROUPS_MAX 24


//
// Quicksort
//

static void __swap (CellDiff *x, CellDiff *y)
{
	CellDiff temp;
	memcpy (&temp, x, sizeof (CellDiff));
	memcpy (x, y, sizeof (CellDiff));
	memcpy (y, &temp, sizeof (CellDiff));
}

static int __choosePivot (const int i, const int j)
{
	return ((i + j) / 2);
}

static void quickSortCellDiffs (CellDiff *cellDiffs, const int start, const int end)
{
	CellDiff key;
	int pivot;
	int i, j;

	if (start < end)
	{
		pivot = __choosePivot (start, end);
		__swap (&cellDiffs[start], &cellDiffs[pivot]);
		memcpy (&key, &cellDiffs[start], sizeof (CellDiff));
		i = start + 1;
		j = end;
		while (i <= j)
		{
			while ((i <= end) && (cellDiffs[i].diff <= key.diff))
			{
				++i;
			}
			while ((j >= start) && (cellDiffs[j].diff > key.diff))
			{
				--j;
			}
			if (i < j)
			{
				__swap (&cellDiffs[i], &cellDiffs[j]);
			}
		}

		// swap two elements
		__swap (&cellDiffs[start], &cellDiffs[j]);

		// recursively sort the lesser list
		quickSortCellDiffs (cellDiffs, start, j - 1);
		quickSortCellDiffs (cellDiffs, j + 1, end);
	}
}

//
// Local functions
//

static void __calcCellDiffs (CellDesc *cellDescs, const int nCells, RefFeats refFeats, CellDiff *diffs)
{
	CellDiff *cf1;
	CellDesc *cd1, *cd2;
	int i, j, k, feat;
	float diff;

	cf1 = diffs;
	cd1 = cellDescs;
	for(i = 0; i < nCells; ++i)
	{
		cd2 = cd1 + 1;
		for (j = i + 1; j < nCells; ++j)
		{
			diff = 0.0f;
			for (k = 0; k < 6; ++k)
			{
				feat = refFeats.indices[k];
				diff += fabs (cd1->feats[k].ratios[feat] - cd2->feats[k].ratios[feat]);
			}
			cf1->diff = diff;
			cf1->a = i;
			cf1->b = j;
			++cf1;
			++cd2;
		}
		++cd1;
	}
}

static int __assignCellGroups (unsigned short *assignedGroups, CellDiff *diffs, const int nCells, const int nDiffs)
{
	CellDiff *cf1;
	int i, nCellsAssigned, groupIndex;

	i = 0;
	groupIndex = 1;
	nCellsAssigned = 0;
	cf1 = diffs;
	while (i < nDiffs && nCellsAssigned < nCells)
	{
		if (assignedGroups[cf1->a] == 0 || assignedGroups[cf1->b] == 0)
		{
			if (assignedGroups[cf1->a] != 0)
			{
				// Add cell b to the group in which cell a is.
				assignedGroups[cf1->b] = assignedGroups[cf1->a];
			}
			else if (assignedGroups[cf1->b] != 0)
			{
				assignedGroups[cf1->a] = assignedGroups[cf1->b];
			}
			else
			{
				assignedGroups[cf1->a] = groupIndex;
				assignedGroups[cf1->b] = groupIndex;
				++groupIndex;
			}
			++nCellsAssigned;
		}

		++cf1;
		++i;
	}

	// Decrement group ids to avoid confusion
	for (i = 0; i < nCells; ++i)
	{
		assignedGroups[i] = assignedGroups[i] - 1;
	}

	return (groupIndex - 1);
}

static void __calcGroupDesc (GroupDesc *groupDesc, const int groupIndex, const unsigned short *assignedGroups, CellDesc *cellDescs, RefFeats refFeats, const int nCells)
{
	int i, j, nCellsThisGroup;
	float f;

	nCellsThisGroup = 0;
	for (i = 0; i < 6; ++i)
	{
		groupDesc->feats[i] = 0.0f;
	}

	for (i = 0; i < nCells; ++i)
	{
		if (assignedGroups[i] == groupIndex)
		{
			++nCellsThisGroup;
			for (j = 0; j < 6; ++j)
			{
				f = cellDescs[i].feats[j].ratios[refFeats.indices[j]];
				groupDesc->feats[j] += f;
			}
		}
	}

	f = 1.0f / nCellsThisGroup;
	for (i = 0; i < 6; ++i)
	{
		groupDesc->feats[i] *= f;
	}
	groupDesc->nCells = nCellsThisGroup;
}

static void __calcGroupDiffs (GroupDesc *groupDescs, GroupDiff *groupDiffs, const int nGroups, const int nGroupDiffs)
{
	GroupDesc *gd1, *gd2;
	GroupDiff *gf1;
	int i, j, k;
	float diff;

	gf1 = groupDiffs;
	gd1 = groupDescs;
	for(i = 0; i < nGroups; ++i)
	{
		gd2 = gd1 + 1;
		for (j = i + 1; j < nGroups; ++j)
		{
			diff = 0.0f;
			for (k = 0; k < 6; ++k)
			{
				diff += fabs (gd1->feats[k] - gd2->feats[k]);
			}
			gf1->diff = diff;
			gf1->a = i;
			gf1->b = j;
			++gf1;
			++gd2;
		}
		++gd1;
	}
}

static void __propagateAssignedGroups (unsigned short *assignedGroups, const unsigned short source, const unsigned short dest, const int nCells)
{
	int i;
	for (i = 0; i < nCells; ++i)
	{
		if (assignedGroups[i] == source)
		{
			assignedGroups[i] = dest;
		}
	}
}

static void __updateGroupDiffs (GroupDiff *groupDiffs, const int currentIndex, const unsigned short source, const unsigned short dest, const int nGroupDiffs)
{
	GroupDiff *gf1;
	int i;

	gf1 = groupDiffs + currentIndex + 1;
	for (i = currentIndex + 1; i < nGroupDiffs; ++i)
	{
		if (gf1->a == source)
		{
			gf1->a = dest;
		}
		if (gf1->b == source)
		{
			gf1->b = dest;
		}
		++gf1;
	}
}

static int __mergeGroups (GroupDiff *groupDiffs, unsigned short *assignedGroups, const int nGroupDiffs, const int nGroups, const int nCells)
{
	GroupDiff *gf1;
	int i, nGroupsFinal;

	i = 0;
	nGroupsFinal = nGroups;
	gf1 = groupDiffs;
	while (i < nGroupDiffs && nGroupsFinal > N_GROUPS_MAX)
	{
		if (gf1->a != gf1->b)
		{
			__propagateAssignedGroups (assignedGroups, gf1->a, gf1->b, nCells);
			__updateGroupDiffs (groupDiffs, i, gf1->a, gf1->b, nGroupDiffs);
			--nGroupsFinal;
			printf ("Merge cell group %d to %d\n", gf1->a, gf1->b);
		}
		++gf1;
		++i;
	}

	return nGroupsFinal;
}

static void __getFinalGroupIndices (unsigned short *finalGroupIndices, const unsigned short *assignedGroups, const int nCells, const int nGroupsFinal)
{
	int i, j, k, groupIndex, isAlreadyIncluded;
	memset (finalGroupIndices, 0xff, sizeof (unsigned short) * nGroupsFinal);
	j = 0;
	for (i = 0; i < nCells; ++i)
	{
		groupIndex = assignedGroups[i];

		isAlreadyIncluded = 0;
		for (k = 0; k < j; ++k)
		{
			isAlreadyIncluded |= (finalGroupIndices[k] == groupIndex);
		}

		if (!isAlreadyIncluded)
		{
			finalGroupIndices[j] = groupIndex;
			++j;
		}
	}
}

//
// Create cell groups
//

//static GroupDesc* __groupCells(CellDesc* cellDescs, int nCells, RefFeats refFeats, int* __nGroups)
static GroupDesc* __groupCells(CellDesc* cellDescs, int nCells, int* __nGroups)
{
	GroupDesc *groupDescs, *finalGroupDescs, *descPtr1, *descPtr2;
	CellDiff *diffs, *diffsPtr;
	GroupDiff *groupDiffs;
	int i, j, nDiffs, nGroups, nGroupsFinal, nGroupDiffs;
	unsigned short *assignedGroups, *finalGroupIndices;
	float diff;
	
	

	// Calculate all cell diffs for this imageGroup
	//
	// If 1000 cells, then first cell will have 999 diffs, the next will have 998, etc. => sum
	// up the numbers to (n - 1). Sum of first n numbers is n(n - 1)/2
	nDiffs = nCells;
	nDiffs = nDiffs * (nDiffs - 1) / 2;
	diffs = (CellDiff*)malloc (sizeof (CellDiff) * nDiffs);

	//__calcCellDiffs (cellDescs, nCells, refFeats, diffs);
	{
		diffsPtr = diffs;
		descPtr1 = cellDescs;
		for (i = 0; i < nCells; ++i)
		{
			descPtr2 = descPtr1 + 1;
			for (j = i + 1; j < nCells; ++j)
			{
				diff =
					fabs(descPtr1[0] - descPtr2[0]) +
					fabs(descPtr1[1] - descPtr2[1]) +
					fabs(descPtr1[2] - descPtr2[2]) +
					fabs(descPtr1[3] - descPtr2[3]) +
					fabs(descPtr1[4] - descPtr2[4]) +
					fabs(descPtr1[5] - descPtr2[5]);

				diffsPtr->diff = diff;
				diffsPtr->id1 = i;
				diffsPtr->type1 = 0; // 0=item 1=group
				diffsPtr->id2 = j;
				diffsPtr->type2 = 0;

				++diffsPtr;
				++descPtr2;
			}
		}
	}
	
	quickSortCellDiffs (diffs, 0, nDiffs - 1);

	create list of indices that haven't been added yet; itemsNotYetAdded
	
	while (nItemsNotYetAdded && (nGroups + nItemsNotYetAdded) > N_CELL_GROUPS_MAX)
	{
		pop tuple at end, i.e. mark as -1
		get item1, item2
		{
			malloc new group
			if (item1.type == 1) // group
			{
				add members to new group members
				mark this group as -1
			}
			else
			{
				just add the id to new group members
				remove from itemsNotYetAdded
			}
			// SAME FOR ITEM 2
			set newGroup id;
			set desc;
			set error;
			create thisItem to comp to;
			for (i = 0; i < nDiffs; ++i)
			{
				check if item1 or item2 in otherDiff;
				set descToCompTo;
				set newError;
				calc newDiff;
				set new values for otherDiff in diffs;
			}
		}

		uniqueify diffs;
		sort diffs;
	}

	create groups for itemsNotYetAdded
	
	
	
	
	// Assign cells to groups. Index starts at 1 s.th. 0 can be used as "no group"
	assignedGroups = (unsigned short*)malloc (sizeof (unsigned short) * nCells);
	memset (assignedGroups, 0, sizeof (unsigned short) * nCells);

	nGroups = __assignCellGroups (assignedGroups, diffs, nCells, nDiffs);

	// Calculate group descriptions
	groupDescs = (GroupDesc*)malloc (sizeof (GroupDesc) * nGroups);
	for (i = 0; i < nGroups; ++i)
	{
		__calcGroupDesc (&groupDescs[i], i, assignedGroups, cellDescs, refFeats, nCells);
	}

	// Calculate simularities between cellGroups
	nGroupDiffs = nGroups;
	nGroupDiffs = nGroupDiffs * (nGroupDiffs - 1) / 2;
	groupDiffs = (GroupDiff*)malloc (sizeof (GroupDiff) * nGroupDiffs);

	__calcGroupDiffs (groupDescs, groupDiffs, nGroups, nGroupDiffs);

	quickSortCellDiffs (groupDiffs, 0, nGroupDiffs - 1);

	// Merge cellGroups.
	//
	// Take sorted list of group similarities. If group a is merged onto group b, update
	// assignedGroups with the new group index for each cell.
	//
	// Also have to update indices in groupDiffs. If a->b, and then a->c, then this second
	// groupDiff should have been updated as b->c.
	nGroupsFinal = __mergeGroups (groupDiffs, assignedGroups, nGroupDiffs, nGroups, nCells);

	finalGroupIndices = (unsigned short*)malloc (sizeof (unsigned short) * nGroupsFinal);
	__getFinalGroupIndices (finalGroupIndices, assignedGroups, nCells, nGroupsFinal);
	
	finalGroupDescs = (GroupDesc*)malloc (sizeof (GroupDesc) * nGroupsFinal);
	for (i = 0; i < nGroupsFinal; ++i)
	{
		__calcGroupDesc (&finalGroupDescs[i], finalGroupIndices[i], assignedGroups, cellDescs, refFeats, nCells);
	}

	free (finalGroupIndices);
	free (groupDiffs);
	free (assignedGroups);
	free (diffs);
	free (groupDescs);

	*__nGroups = nGroupsFinal;

	return finalGroupDescs;
}

//
// Module functions
//

static PyObject* imageUtils_groupCells(PyObject *self, PyObject *args)
{
	int res;
	PyObject *list;
	PyObject *list2;
	//PyObject *list3;
	//PyObject *refFeatsList;
	PyObject *floatObj;
	PyObject *intObj;
	PyObject *dict;
	PyObject *key1;
	PyObject *key2;
	CellDesc *cellDescs;
	GroupDesc *groupDescs;
	RefFeats refFeats;
	int i, j, k;
	int nCells, nGroups;

	// Get values from Python
	res = PyArg_ParseTuple(args, "OO", &list);
	if (!res)
	{
		printf("Error\n");
		Py_INCREF(Py_None);
		return Py_None;
	}

	nCells = PyList_Size(list);
	printf("Creating cellGroups from %d cells\n", nCells);

	cellDescs = (CellDesc*)malloc (sizeof (CellDesc) * nCells);

	for(i = 0; i < nCells; ++i)
	{
		list2 = PyList_GetItem(list, i);

		j = PyList_Size(list2);
		if (j != 6)
		{
			printf ("Wrong list size %d %d (iter %d)\n", j, 6, i);
			assert(0);
		}

		for(j = 0; j < 6; ++j)
		{
			//list3 = PyList_GetItem (list2, j);
			floatObj = PyList_GetItem(list2, j);
			cellDescs[i].feats[j] = PyFloat_AsDouble(floatObj);

			/*k = PyList_Size (list3);
			if (k != 12)
			{
				printf ("Wrong list size %d %d (iter %d %d)\n", k, 12, i, j);
				assert(0);
			}

			for (k = 0; k < 12; ++k)
			{
				floatObj = PyList_GetItem(list3, k);
				cellDescs[i].feats[j].ratios[k] = PyFloat_AsDouble(floatObj);
			}*/
		}
	}

	// Group cellDescs to create groupDescs
	//groupDescs = __groupCells (cellDescs, nCells, refFeats, &nGroups);
	groupDescs = __groupCells (cellDescs, nCells, &nGroups);

	// Convert values to Python objects
	list = PyList_New(nGroups);
	key1 = PyUnicode_FromString ("nCells");
	key2 = PyUnicode_FromString ("desc");
	for(i = 0; i < nGroups; ++i)
	{
		dict = PyDict_New();
		intObj = PyLong_FromLong(groupDescs[i].nCells);
		PyDict_SetItem (dict, key1, intObj);

		list2 = PyList_New(6);
		for (j = 0; j < 6; ++j)
		{
			floatObj = PyFloat_FromDouble (groupDescs[i].feats[j]);
			PyList_SetItem (list2, j, floatObj);
		}

		PyDict_SetItem (dict, key2, list2);
		PyList_SetItem(list, i, dict);
	}

	free (cellDescs);
	free (groupDescs);

	return list;
}

static PyMethodDef imageUtils_methods[] = {
	{"groupCells", imageUtils_groupCells, METH_VARARGS, "groupCells() doc string"},
	{NULL, NULL}
};

static struct PyModuleDef imageUtilsModule = {
	PyModuleDef_HEAD_INIT,
	"imageUtils",
	"imageUtils module doc string",
	-1,
	imageUtils_methods,
	NULL,
	NULL,
	NULL,
	NULL
};

PyMODINIT_FUNC
PyInit_imageUtils(void)
{
	return PyModule_Create(&imageUtilsModule);
}
