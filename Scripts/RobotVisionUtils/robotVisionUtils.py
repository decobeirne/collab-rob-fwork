import sys
import os.path
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir))
import pickle
import math
from pdb import set_trace
import utils
from pprint import pprint
from array import array


LOG = utils.log
WRAP_FUNCTION = utils.wrapFunction

N_IMG_FEATS = 12
OCCUP_GRID_X_DIMS = 19
OCCUP_GRID_Y_DIMS = 23
N_IMAGE_GROUPS_MAX = 20
N_CELL_GROUPS_MAX = 32
N_CELLS_PER_BLOCK = 600 # 1000
N_CELLS_TO_SPLIT_INTO_BLOCKS = N_CELLS_PER_BLOCK * 1.5
ON_LAPTOP = 1
if ON_LAPTOP:
    ROOT_DIR = r'D:\thesis'
else:
    ROOT_DIR = r'C:\stuff\t'
OBST_REC_DATA_DIR = os.path.normpath(os.path.join(ROOT_DIR, r'calibration\ObstacleRecognition'))
ROBOT_REC_DATA_DIR = os.path.normpath(os.path.join(ROOT_DIR, r'calibration\RobotRecognition'))
ROBOT_REC_COLOURS_FILE = 'coloursToUse.txt'
IMAGE_GROUPS_PICKLE = '__imageGroups__.p'
IMAGE_STDDEVS_PICKLE = '__imageStdDevs__.p'
COLOUR_GROUPS_PICKLE = '__colourGroups_.p'
COLOUR_AVGS_PICKLE = '__colourAvgs_.p'
CELL_DIFF_COEFF = 1.0
IMAGE_DIFF_COEFF = 2.0  # TODO: arbitrary

# (2013/02/03) from testing (in script), only a tiny fraction of estimates were incorrectd - 20 out of 40000.
# Ratio threshold of 1.5 should be safe, but could go with 2 => consider everything less than this as
# unoccupied in nav map, but only submit ratio greater than 1.5
OCCUPANCY_CONFIDENCE_THRESHOLD = 1.5
COLOUR_TESTING_THRESHOLD = 3.0


def getCellDescriptionFilesInDir(dataDir):
    return [x for x in os.listdir(dataDir) if 'cellFeatures' in x]

def getColourDescriptionFilesInDir(dataDir):
    return [x for x in os.listdir(dataDir) if 'colourFeatures' in x]

def loadColourDescriptionsInFile(dataDir, fileName):
    colourFile = open(os.path.join(dataDir, fileName))
    colourFileLines = [x for x in colourFile.readlines() if x != '']
    nLines = len(colourFileLines)
    assert nLines % 8 == 0, 'Unexpected number of lines in file %s' % colourFilename
    nFeats = nLines // 8

    colourFeats = []
    lineIndex = 0
    for i in range(nFeats):
        imgId = int(colourFileLines[lineIndex])
        cellId = int(colourFileLines[lineIndex + 1])
        f0 = float(colourFileLines[lineIndex + 2])
        f1 = float(colourFileLines[lineIndex + 3])
        f2 = float(colourFileLines[lineIndex + 4])
        f3 = float(colourFileLines[lineIndex + 5])
        f4 = float(colourFileLines[lineIndex + 6])
        f5 = float(colourFileLines[lineIndex + 7])
        cellDesc = (imgId, cellId, (f0, f1, f2, f3, f4, f5))
        colourFeats.append(cellDesc)

        lineIndex += 8
    return colourFeats
    
def loadImageDescriptionInFile(dataDir, fileName):
    imageFeats = open(os.path.join(dataDir, fileName)).readlines()
    imageFeats = [float(y) for y in [x.strip() for x in imageFeats] if y != '']
    return imageFeats

def loadCellDescriptionsInFile(dataDir, fileName, isForTraining):
    cellList = []
    cellFeatsRawList = open(os.path.join(dataDir, fileName)).readlines()
    cellFeatsList = [y for y in (x.strip() for x in cellFeatsRawList) if y != '']
    nCellsThisImage = OCCUP_GRID_X_DIMS * int(cellFeatsList[0]) # Read occupancy grid Y dimensions from start of list
    for cellIndex in range(0, nCellsThisImage):
        cellDesc = [0] * 6
        listIndex = 1 + cellIndex * 7
        isOccupied = int(cellFeatsList[listIndex])
        cellDesc[0] = float(cellFeatsList[listIndex+1])
        cellDesc[1] = float(cellFeatsList[listIndex+2])
        cellDesc[2] = float(cellFeatsList[listIndex+3])
        cellDesc[3] = float(cellFeatsList[listIndex+4])
        cellDesc[4] = float(cellFeatsList[listIndex+5])
        cellDesc[5] = float(cellFeatsList[listIndex+6])
        
        if isOccupied == 2 and isForTraining: # Cell had been marked as unknown in the training data
            continue
        if isForTraining:
            cellList.append(cellDesc)
        else:
            cellList.append((isOccupied, cellDesc))
    return cellList

def getNObstRecTrainingImages(dataDir):
    nImages = len([x for x in os.listdir(dataDir) if x.startswith('cellFeatures')])
    nImages != 0, 'ERROR: nImages not set or no images found'
    return nImages

def getNRobotRecTrainingImages(dataDir):
    nImages = len([x for x in os.listdir(dataDir) if x.startswith('colourFeatures')])
    nImages != 0, 'ERROR: nImages not set or no images found'
    return nImages

def readColoursToUse(coloursFilename):
    assert os.path.isfile(coloursFilename), 'Cannot find colours txt file'
    colours = [int(x) for x in open(coloursFilename).readlines() if x != '']
    return colours

def pickleItem(imageGroups, pickleFilename):
    pickleFile = open(pickleFilename, 'wb')
    pickle.dump(imageGroups, pickleFile, protocol=2)

def loadPickledItem(pickleFilename):
    pickleFile = open(pickleFilename, 'rb')
    imageGroups = pickle.load(pickleFile)
    return imageGroups

def createGroups(items, calcGroupDescFunc, calcGroupErrorFunc, calcItemDiffFunc, calcGroupDiffFunc, calcItemGroupDiffFunc, getGroupFromSingleItemFunc, nGroupsMax, outputFile):
    # Setup diffs between items
    diffDicts = {}
    absFunc = math.fabs
    nItems = len(items)
    LOG('%d items to group' % nItems, file=outputFile)
    for itemIndex in range(nItems):
        thisItem = items[itemIndex]
        diffsThisItem = {}
        for otherIndex in range(itemIndex + 1, nItems):
            otherItem = items[otherIndex]
            diff = calcItemDiffFunc(thisItem, otherItem)
            otherKey = 1000000 + otherIndex  # 1000000=item 2000000=group, use integer arithmetic to get type from key
            diffsThisItem[otherKey] = (diff, 0)
        thisKey = 1000000 + itemIndex
        diffDicts[thisKey] = diffsThisItem
        if diffsThisItem:
            closestKey = min(diffsThisItem.items(), key=lambda x: (x[1][0] + x[1][1]))[0]  # items() will return tuples of the form (key, value), so take item [0]
            closestValue = diffsThisItem[closestKey][0] + diffsThisItem[closestKey][1]
            diffsThisItem['closestMatch'] = (99999, 99999, closestValue, closestKey, thisKey)  # We only access items 0 and 1 for regular tuples
        else:
            # Just add this to avoid having to check for exceptions in the loop below; last cell will have no diffs
            diffsThisItem['closestMatch'] = (99999, 99999, 99999, -1, thisKey)

    # Create groups based on diffs between items
    groups = {}
    nextGroupId = 0
    itemsNotYetGrouped = [x for x in range(len(items))]
    while (len(groups) + len(itemsNotYetGrouped)) > nGroupsMax:
    
        # Re-calc which entity has the best match to another entity
        closestMatch = min((diffDicts[x]['closestMatch'] for x in diffDicts), key=lambda x: x[2])
        LOG('(%d left) Creating group %3d from diff: %s' % (len(itemsNotYetGrouped), nextGroupId, repr(closestMatch[2:])), file=outputFile)
        keyI = closestMatch[4]
        keyJ = closestMatch[3]
        typeI = keyI // 1000000  # 1000000=item 2000000=group
        typeJ = keyJ // 1000000
        idI = keyI % 1000000
        idJ = keyJ % 1000000
        
        # Remove diffs that i, j had with everything else
        if keyI in diffDicts:
            diffDicts.pop(keyI)
        if keyJ in diffDicts:
            diffDicts.pop(keyJ)
            
        # Remove diffs that everything else had with i, j
        otherKeys = list(diffDicts.keys())
        for otherKey in otherKeys:
            otherDict = diffDicts[otherKey]
            dirty = otherDict['closestMatch'][3] in (keyI, keyJ)
            if keyI in otherDict:
                otherDict.pop(keyI)
                if len(otherDict) == 1:
                    diffDicts.pop(otherKey)
                    continue
            if keyJ in otherDict:
                otherDict.pop(keyJ)
                if len(otherDict) == 1:
                    diffDicts.pop(otherKey)
                    continue

            # If closest match had been with either i or j, then recalc
            if dirty:
                closestKey = min(otherDict.items(), key=lambda x: (x[1][0] + x[1][1]))[0]
                closestValue = otherDict[closestKey][0] + otherDict[closestKey][1]
                otherDict['closestMatch'] = (99999, 99999, closestValue, closestKey, otherKey)
        
        # Create new group
        newGroup = {}
        if typeI == 2: 
            newGroup['members'] = groups[idI]['members']
            groups.pop(idI)
        else:
            newGroup['members'] = [idI]
            itemsNotYetGrouped.remove(idI)
        if typeJ == 2:
            newGroup['members'].extend(groups[idJ]['members'])
            groups.pop(idJ)
        else:
            newGroup['members'].append(idJ)
            itemsNotYetGrouped.remove(idJ)
        newGroup['id'] = nextGroupId
        newKey = 2000000 + nextGroupId
        groups[nextGroupId] = newGroup
        nextGroupId += 1
        newGroup['desc'] = calcGroupDescFunc(newGroup['members'], items)
        newGroup['error'] = calcGroupErrorFunc(newGroup['members'], newGroup['desc'], items)

        # Create new dictDiff
        newDictDiff = {}
        for otherKey in diffDicts.keys():
            otherType = otherKey // 1000000
            otherId = otherKey % 1000000
            if otherType == 2:
                newError = newGroup['error'] + groups[otherId]['error']
                newDiff = calcGroupDiffFunc(newGroup['desc'], groups[otherId]['desc'])
            else:
                newError = newGroup['error']
                newDiff = calcItemGroupDiffFunc(items[otherId], newGroup['desc'])
            newDictDiff[otherKey] = (newDiff, newError)
        closestKey = min(newDictDiff.items(), key=lambda x: (x[1][0] + x[1][1]))[0]
        closestValue = newDictDiff[closestKey][0] + newDictDiff[closestKey][1]
        newDictDiff['closestMatch'] = (99999, 99999, closestValue, closestKey, newKey)
        diffDicts[newKey] = newDictDiff

    # Put any remaining items into groups on their own
    LOG('Items not joined to any groups:', file=outputFile)
    pprint(itemsNotYetGrouped, stream=outputFile)
    for id in itemsNotYetGrouped:
        groupDesc = getGroupFromSingleItemFunc(items[id])
        newGroup = {'members': [id], 'desc': groupDesc, 'error':0}
        newGroup['id'] = nextGroupId
        groups[nextGroupId] = newGroup
        nextGroupId += 1

    LOG('%d groups:' % len(groups), file=outputFile)
    for groupKey in groups:
        group = groups[groupKey]
        LOG('Group %d (%d)' % (group['id'], len(group['members'])), file=outputFile)
        LOG(repr(group['members']), file=outputFile)
        group.pop('id')
    
    return groups

def createGroupsNEW(items, calcGroupDescFunc, calcGroupErrorFunc, calcItemDiffFunc, calcGroupDiffFunc, calcItemGroupDiffFunc, getGroupFromSingleItemFunc, nGroupsMax, outputFile):

    absFunc = math.fabs
    nItems = len(items)
    LOG('%d items to group' % nItems)
    LOG('%d items to group' % nItems, file=outputFile)

    def __getCellCellClosestMatch(cellDiffs, itemIndex):
        diffsThisCell = cellDiffs[itemIndex]
        nDiffs = len(diffsThisCell)
        closestDiff = 99999
        for otherIndex in range(nDiffs):
            actualOtherIndex = itemIndex + 1 + otherIndex
            if cellDiffs[actualOtherIndex] is not None:  # Cell already removed and put in a group
                thisDiff = diffsThisCell[otherIndex]
                if thisDiff < closestDiff:
                    closestDiff = thisDiff
                    closestIndex = actualOtherIndex
        
        if closestDiff != 99999:
            closestMatch = (99999, 99999, closestDiff, 1000000 + closestIndex, 1000000 + itemIndex)  # We only access items 0 and 1 for regular tuples
        else:
            closestMatch = (99999, 99999, 99999, -1, 1000000 + itemIndex)
        return closestMatch

    def __getGroupCellClosestMatch(cellDiffs, groupCellDiffsThisGroup, groupError, groupIndex):
        closestDiff = 99999
        for itemIndex in range(len(cellDiffs)):
            if cellDiffs[itemIndex] is not None:
                cellDiff = groupCellDiffsThisGroup[itemIndex]
                if cellDiff < closestDiff:
                    closestDiff = cellDiff
                    closestKey = itemIndex
        if closestDiff != 99999:
            closestMatch = (99999, 99999, closestDiff + groupError, 1000000 + closestKey, 2000000 + groupIndex)
        else:
            closestMatch = (99999, 99999, 99999, -1, 2000000 + groupIndex)
        return closestMatch

    def __getGroupGroupClosestMatch(groupGroupDiffs, groupErrors, groupIndex):
        groupDiffsThisGroup = groupGroupDiffs[groupIndex]
        groupErrorThisGroup = groupErrors[groupIndex]
        closestDiff = 99999
        for otherIndex in range(len(groupDiffsThisGroup)):
            if groupGroupDiffs[otherIndex] is not None:
                groupDiff = groupDiffsThisGroup[otherIndex]
                finalDiff = groupDiff + groupErrors[otherIndex] + groupErrorThisGroup
                if finalDiff < closestDiff:
                    closestDiff = finalDiff
                    closestKey = otherIndex
        if closestDiff != 99999:
            closestMatch = (99999, 99999, closestDiff, 2000000 + closestKey, 2000000 + groupIndex)
        else:
            closestMatch = (99999, 99999, 99999, -1, 2000000 + groupIndex)
        return closestMatch

    # Setup diffs between items
    cellDiffs = []
    cellClosestMatches = []
    for itemIndex in range(nItems):
        thisItem = items[itemIndex]

        diffsThisCell = array('f')
        cellDiffs.append(diffsThisCell)
        closestDiff = 99999
        for otherIndex in range(itemIndex + 1, nItems):
            otherItem = items[otherIndex]
            diff = calcItemDiffFunc(thisItem, otherItem)
            diffsThisCell.append(diff)
            if diff < closestDiff:
                closestDiff = diff
                closestKey = otherIndex
            
        thisKey = 1000000 + itemIndex
        if closestDiff != 99999:
            closestMatch = (99999, 99999, closestDiff, 1000000 + closestKey, thisKey)
        else:
            # Just add this to avoid having to check for exceptions in the loop below; last cell will have no diffs
            closestMatch = (99999, 99999, 99999, -1, thisKey)
        cellClosestMatches.append(closestMatch)

    # Create groups based on diffs between items
    nGroups = 0
    groupDescs = []
    groupErrors = []
    groupMembers = []
    groupCellDiffs = []  # Quicker to add 1 array of len nCells, than add 1 item to nCells arrays
    groupCellClosestMatches = []
    groupGroupDiffs = []
    groupGroupClosestMatches = []
    itemsNotYetGrouped = [x for x in range(len(items))]
    while (nGroups + len(itemsNotYetGrouped)) > nGroupsMax:

        # Re-calc which entity has the best match to another entity
        closestCellCellMatch = min(cellClosestMatches, key=lambda x: x[2])
        closestGroupCellMatch = min(groupCellClosestMatches, key=lambda x: x[2]) if groupCellClosestMatches else (99999, 99999, 99999, 99999, 99999)
        closestGroupGroupMatch = min(groupGroupClosestMatches, key=lambda x: x[2]) if groupGroupClosestMatches else (99999, 99999, 99999, 99999, 99999)
        closestMatch = min(closestCellCellMatch, min(closestGroupCellMatch, closestGroupGroupMatch, key=lambda x: x[2]), key=lambda x: x[2])

        LOG('(%d left) Creating group %3d from diff: %s' % (len(itemsNotYetGrouped), len(groupDescs), repr(closestMatch[2:])), file=outputFile)
        keyI = closestMatch[4]
        keyJ = closestMatch[3]
        typeI = keyI // 1000000  # 1000000=item 2000000=group
        typeJ = keyJ // 1000000
        idI = keyI % 1000000
        idJ = keyJ % 1000000
        
        # Remove diffs that i, j had with everything else
        if typeI == 1:
            cellDiffs[idI] = None
            cellClosestMatches[idI] = (99999, 99999, 99999, -1, keyI)
        else:
            groupCellDiffs[idI] = None
            groupCellClosestMatches[idI] = (99999, 99999, 99999, -1, keyI)
            groupGroupDiffs[idI] = None
            groupGroupClosestMatches[idI] = (99999, 99999, 99999, -1, keyI)
        if typeJ == 1:
            cellDiffs[idJ] = None
            cellClosestMatches[idJ] = (99999, 99999, 99999, -1, keyJ)
        else:
            groupCellDiffs[idJ] = None
            groupCellClosestMatches[idJ] = (99999, 99999, 99999, -1, keyJ)
            groupGroupDiffs[idJ] = None
            groupGroupClosestMatches[idJ] = (99999, 99999, 99999, -1, keyJ)
        
        # Remove diffs that everything else had with i, j
        if typeI == 1:
            # if idI = 200 => only 0..199 have a diff to it
            for itemIndex in range(idI):
                # if idI == 200, itemIndex == 199 => (200 - 199 - 1)
                indexInListI = idI - itemIndex - 1
                if cellDiffs[itemIndex]:
                    cellDiffs[itemIndex][indexInListI] = 99999
                if cellClosestMatches[itemIndex][3] == keyI:
                    # Have to recalc closest cell match for this cell
                    cellClosestMatches[itemIndex] = __getCellCellClosestMatch(cellDiffs, itemIndex)

            for groupIndex in range(len(groupDescs)):
                if groupCellDiffs[groupIndex]:
                    groupCellDiffs[groupIndex][idI] = 99999
                if groupCellClosestMatches[groupIndex][3] == keyI:
                    groupCellClosestMatches[groupIndex] = __getGroupCellClosestMatch(cellDiffs, groupCellDiffs[groupIndex], groupErrors[groupIndex], groupIndex)
        else:
            # When we add group 10, we calc diffs to groups 0..9, so if we delete 10, we only have to
            # delete references to it in groups 11..nGroups
            for groupIndex in range(idI + 1, len(groupDescs)):
                if keyI == 2000006 and groupIndex == 2001045:
                    set_trace()
                # Group-group diffs for groupX are for group ids 0..groupX
                if groupGroupDiffs[groupIndex]:
                    groupGroupDiffs[groupIndex][idI] = 99999
                if groupGroupClosestMatches[groupIndex][3] == keyI:
                    groupGroupClosestMatches[groupIndex] = __getGroupGroupClosestMatch(groupGroupDiffs, groupErrors, groupIndex)
        
        if typeJ == 1:
            for itemIndex in range(idJ):
                indexInListJ = idJ - itemIndex - 1
                if cellDiffs[itemIndex]:
                    cellDiffs[itemIndex][indexInListJ] = 99999
                if cellClosestMatches[itemIndex][3] == keyJ:
                    cellClosestMatches[itemIndex] = __getCellCellClosestMatch(cellDiffs, itemIndex)

            for groupIndex in range(len(groupDescs)):
                if groupCellDiffs[groupIndex]:
                    groupCellDiffs[groupIndex][idJ] = 99999
                if groupCellClosestMatches[groupIndex][3] == keyJ:
                    groupCellClosestMatches[groupIndex] = __getGroupCellClosestMatch(cellDiffs, groupCellDiffs[groupIndex], groupErrors[groupIndex], groupIndex)
        else:
            for groupIndex in range(idJ + 1, len(groupDescs)):
                if groupGroupDiffs[groupIndex]:
                    groupGroupDiffs[groupIndex][idJ] = 99999
                if groupGroupClosestMatches[groupIndex][3] == keyJ:
                    groupGroupClosestMatches[groupIndex] = __getGroupGroupClosestMatch(groupGroupDiffs, groupErrors, groupIndex)

        # Create new group
        nGroups += 1
        if typeI == 2:
            nGroups -= 1
            newGroupMembers = groupMembers[idI]
            groupMembers.append(newGroupMembers)
            groupMembers[idI] = None
        else:
            newGroupMembers = array('i')
            newGroupMembers.append(idI)
            groupMembers.append(newGroupMembers)
            itemsNotYetGrouped.remove(idI)
        if typeJ == 2:
            nGroups -= 1
            newGroupMembers.extend(groupMembers[idJ])
            groupMembers[idJ] = None
        else:
            newGroupMembers.append(idJ)
            itemsNotYetGrouped.remove(idJ)

        newKey = 2000000 + len(groupDescs)
        newGroupDesc = calcGroupDescFunc(newGroupMembers, items)
        groupDescs.append(newGroupDesc)
        newGroupError = calcGroupErrorFunc(newGroupMembers, newGroupDesc, items)
        groupErrors.append(newGroupError)

        # Setup diffs for the new group - to cells and to existing groups
        diffsThisGroup = array('f')
        groupCellDiffs.append(diffsThisGroup)
        closestDiff = 99999
        for itemIndex in range(nItems):
            if cellDiffs[itemIndex] is None:  # Make sure we explicitly check is None, as it may be []
                diffsThisGroup.append(99999)
            else:
                cellDiff = calcItemGroupDiffFunc(items[itemIndex], newGroupDesc)
                diffsThisGroup.append(cellDiff)
                if cellDiff < closestDiff:
                    closestDiff = cellDiff
                    closestKey = itemIndex

        if closestDiff != 99999:
            closestMatch = (99999, 99999, closestDiff + newGroupError, 1000000 + closestKey, newKey)
        else:
            closestMatch = (99999, 99999, 99999, -1, newKey)
        groupCellClosestMatches.append(closestMatch)
        
        groupDiffsThisGroup = array('f')
        groupGroupDiffs.append(groupDiffsThisGroup)
        closestDiff = 99999
        for groupIndex in range(len(groupDescs) - 1):  # Desc for new group is in the last spot in this list
            if groupGroupDiffs[groupIndex] is None:
                groupDiffsThisGroup.append(99999)
            else:
                groupDiff = calcGroupDiffFunc(groupDescs[groupIndex], newGroupDesc)
                groupDiffsThisGroup.append(groupDiff)
                finalDiff = groupDiff + groupErrors[groupIndex] + newGroupError
                if finalDiff < closestDiff:
                    closestDiff = finalDiff
                    closestKey = groupIndex

        if closestDiff != 99999:
            closestMatch = (99999, 99999, closestDiff, 2000000 + closestKey, newKey)
        else:
            closestMatch = (99999, 99999, 99999, -1, newKey)
        groupGroupClosestMatches.append(closestMatch)
    
    # Put any remaining items into groups on their own
    LOG('Items not joined to any groups:', file=outputFile)
    pprint(itemsNotYetGrouped, stream=outputFile)
    for id in itemsNotYetGrouped:
        newGroupDesc = getGroupFromSingleItemFunc(items[id])
        newGroupMembers = array('f')
        newGroupMembers.append(id)
        newGroupError = 0
        groupDescs.append(newGroupDesc)
        groupMembers.append(newGroupMembers)
        groupErrors.append(newGroupError)

    LOG('%d groups:' % len(groupDescs), file=outputFile)
    groups = []
    for groupIndex in range(len(groupDescs)):
        if groupGroupDiffs[groupIndex] is not None:
            LOG('Group %d (%d)' % (groupIndex, len(groupMembers[groupIndex])), file=outputFile)
            LOG(repr(groupMembers[groupIndex]), file=outputFile)
            groupRep = {'desc': groupDescs[groupIndex], 'members': groupMembers[groupIndex], 'error': groupErrors[groupIndex]}
            groups.append(groupRep)
    LOG('%d actual groups:' % len(groups), file=outputFile)

    return groups

