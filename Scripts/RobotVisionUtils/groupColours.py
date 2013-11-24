import sys
import os.path
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir))
from math import fabs
from pprint import pprint
from pdb import set_trace
import robotVisionUtils
from utils import log, wrapFunction
from gc import collect

LOG = log
WRAP_FUNCTION = wrapFunction

def calcColourFeats(colourFileLines, colourFilename):
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

def calcColourCellDiff(cell1, cell2):
    # Colour cells have the format (imgId, cellId, description)
    absFunc = fabs
    diff = absFunc(cell1[2][0] - cell2[2][0]) + \
           absFunc(cell1[2][1] - cell2[2][1]) + \
           absFunc(cell1[2][2] - cell2[2][2]) + \
           absFunc(cell1[2][3] - cell2[2][3]) + \
           absFunc(cell1[2][4] - cell2[2][4]) + \
           absFunc(cell1[2][5] - cell2[2][5]);
    return diff

def calcColourCellGroupDiff(cell1, group1):
    # Colour cells have the format (imgId, cellId, description)
    absFunc = fabs
    diff = absFunc(cell1[2][0] - group1[0]) + \
       absFunc(cell1[2][1] - group1[1]) + \
       absFunc(cell1[2][2] - group1[2]) + \
       absFunc(cell1[2][3] - group1[3]) + \
       absFunc(cell1[2][4] - group1[4]) + \
       absFunc(cell1[2][5] - group1[5]);
    return diff

def calcColourGroupDiff(group1, group2):
    # Colour cells have the format (imgId, cellId, description)
    absFunc = fabs
    diff = absFunc(group1[0] - group2[0]) + \
           absFunc(group1[1] - group2[1]) + \
           absFunc(group1[2] - group2[2]) + \
           absFunc(group1[3] - group2[3]) + \
           absFunc(group1[4] - group2[4]) + \
           absFunc(group1[5] - group2[5]);
    return diff

def calculateColourCellGroupDescription(groupMembers, items):
    groupDesc = [0] * 6
    for itemIndex in groupMembers:
        itemDesc = items[itemIndex]
        # Colour cells have the format (imgId, cellId, description)
        groupDesc[0] += itemDesc[2][0]
        groupDesc[1] += itemDesc[2][1]
        groupDesc[2] += itemDesc[2][2]
        groupDesc[3] += itemDesc[2][3]
        groupDesc[4] += itemDesc[2][4]
        groupDesc[5] += itemDesc[2][5]
    nGroupMembersInv = 1 / len(groupMembers)
    desc = [x * nGroupMembersInv for x in groupDesc]
    return desc

def calculateColourCellGroupError(groupMembers, groupDesc, items):
    groupError = 0
    absFunc = fabs
    for groupMember in groupMembers:
        member = items[groupMember]
        diff = absFunc(member[2][0] - groupDesc[0]) + \
               absFunc(member[2][1] - groupDesc[1]) + \
               absFunc(member[2][2] - groupDesc[2]) + \
               absFunc(member[2][3] - groupDesc[3]) + \
               absFunc(member[2][4] - groupDesc[4]) + \
               absFunc(member[2][5] - groupDesc[5]);
        groupError += diff

    #groupError *= (2 / len(groupMembers))
    #groupError /= len(groupMembers)
    return groupError

def getColourGroupFromSingleCell(cellDesc):
    groupDesc = cellDesc[2]
    return groupDesc

@WRAP_FUNCTION
def calcColourGroups(colourCells, colour, outputFile):
    '''Wrapper function for calling generic calcGroups function.'''
    LOG('Creating group for colour %d' % colour, file=outputFile)
    groups = robotVisionUtils.createGroupsNEW(
        items = colourCells,
        calcGroupDescFunc = calculateColourCellGroupDescription,
        calcGroupErrorFunc = calculateColourCellGroupError,
        calcItemDiffFunc = calcColourCellDiff,
        calcGroupDiffFunc = calcColourGroupDiff,
        calcItemGroupDiffFunc = calcColourCellGroupDiff,
        getGroupFromSingleItemFunc = getColourGroupFromSingleCell,
        nGroupsMax = robotVisionUtils.N_CELL_GROUPS_MAX,
        outputFile=outputFile)
    return groups

@WRAP_FUNCTION
def checkColourGroups(groups, items, colour, outputFile):
    '''
    Sanity check to verify that the groups make sense
    '''
    if not items:
        LOG('No items to check', file=outputFile)
        return 0, 0

    LOG('Compare item descriptions to groups for colour %d' % colour, file=outputFile)
    absFunc = fabs
    nWrong = 0
    nGroups = len(groups)
    for i in range(0, len(items)):
        itemDesc = items[i]
        diffsStr = ['Cell %d: ' % i]
        minDiff = 10000
        groupDiff = 10000
        minIndex = -1
        groupIndex = -1
        #for key, group in groups.items():
        for groupIndex in range(nGroups):
            group = groups[groupIndex]
            groupDesc = group['desc']
            diff = calcColourCellGroupDiff(itemDesc, groupDesc)
            inGroup = 0
            if i in group['members']:
                #groupIndex = key
                groupDiff = diff
                inGroup = 1
            if diff < minDiff:
                minDiff = diff
                #minIndex = key
                minIndex = groupIndex
            minDiff = min(diff, minDiff)
            diffsStr.append(' %3d%s' % (int(diff), '~' if inGroup else ','))
        diffsStr.append(' min=%3d(%3d) group=%3d(%3d)' % (minIndex, minDiff, groupIndex, groupDiff))
        if minIndex != groupIndex:
            diffsStr.append(' WRONG:%d' % (groupDiff - minDiff))
            LOG(''.join(diffsStr), file=outputFile)
            nWrong+=1
        #break ####################################################################
    LOG('COMP_CELL_TO_GROUPS nWrong=%d nCells=%d %%=%f' % (nWrong, len(items), nWrong/len(items)), file=outputFile)

    LOG('Group errors', file=outputFile)
    groupErrorList = [x['error'] for x in groups]
    totalGroupError = sum(groupErrorList)
    LOG(repr(groupErrorList), file=outputFile)
    LOG('CELL_GROUP_ERROR sum=%f avg=%f' % (totalGroupError, sum(groupErrorList) / len(groupErrorList)), file=outputFile)
    '''LOG('Group errors', file=outputFile)
    groupErrorList = [groups[x]['error'] for x in groups if groups[x]]
    totalGroupError = sum(groupErrorList)
    LOG(repr(groupErrorList), file=outputFile)
    LOG('CELL_GROUP_ERROR sum=%f avg=%f' % (totalGroupError, sum(groupErrorList) / len(groupErrorList)), file=outputFile)'''
    
    
    
    LOG('Compare groups to eachother:', file=outputFile)
    nGroups = len(groups)
    #nKeys = len(keys)
    groupsStr = [' %3d,' % x for x in range(1, nGroups)]
    LOG('   ' + ''.join(groupsStr), file=outputFile)
    LOG('', file=outputFile)
    smallDiffs = []
    #for i in range(0, nKeys):
    for i in range(nGroups):
        diffsStr = []
        diffsStr.append('%3d:' % i + '_' * (5 * i))
        for j in range(i + 1, nGroups):
            descI = groups[i]['desc']
            descJ = groups[j]['desc']
            diff = calcColourGroupDiff(descI, descJ)
            diffsStr.append(' %3d,' % diff)
            if diff < 6:
                smallDiffs.append((i, j, diff))
        LOG(''.join(diffsStr), file=outputFile)
    LOG('%d Cell groups are too similar:' % len(smallDiffs), file=outputFile)
    for i, j, diff in smallDiffs:
        LOG('Diff between %d and %d = %d' % (i, j, diff), file=outputFile)
        pprint(groups[i]['desc'], stream=outputFile)
        pprint(groups[j]['desc'], stream=outputFile)
    '''LOG('Compare groups to eachother:', file=outputFile)
    keys = list(groups.keys())
    nKeys = len(keys)
    groupsStr = [' %3d,' % x for x in keys[1:]]
    LOG('   ' + ''.join(groupsStr), file=outputFile)
    LOG('', file=outputFile)
    smallDiffs = []
    for i in range(0, nKeys):
        diffsStr = []
        diffsStr.append('%3d:' % keys[i] + '_' * (5 * i))
        for j in range(i + 1, nKeys):
            descI = groups[keys[i]]['desc']
            descJ = groups[keys[j]]['desc']
            diff = calcColourGroupDiff(descI, descJ)
            diffsStr.append(' %3d,' % diff)
            if diff < 6:
                smallDiffs.append((keys[i], keys[j], diff))
        LOG(''.join(diffsStr), file=outputFile)
    LOG('%d Cell groups are too similar:' % len(smallDiffs), file=outputFile)
    for i, j, diff in smallDiffs:
        LOG('Diff between %d and %d = %d' % (i, j, diff), file=outputFile)
        pprint(groups[i]['desc'], stream=outputFile)
        pprint(groups[j]['desc'], stream=outputFile)'''
    
    return (nWrong, totalGroupError)

def checkAllColourGroups(colourGroupDict, colourCellsDict, outputFile):

    LOG('Compare item descriptions to groups for all colours', file=outputFile)
    absFunc = fabs
    nWrong = 0
    colourCellsKeys = sorted(colourCellsDict.keys())
    colourGroupKeys = sorted(colourGroupDict.keys())
    for colourCellsKey in colourCellsKeys:
        colourCells = colourCellsDict[colourCellsKey]
        for colourGroupKey in colourGroupKeys:
            LOG('Comparing cells for colour %d to groups for %d' % (colourCellsKey, colourGroupKey), file=outputFile)
            colourGroups = colourGroupDict[colourGroupKey]

            for i in range(0, len(colourCells)):
                itemDesc = colourCells[i]
                diffsStr = ['Cell %d: ' % i]
                minDiff = 10000
                groupDiff = 10000
                minIndex = -1
                groupIndex = -1
                for key, group in colourGroups.items():
                    groupDesc = group['desc']
                    diff = calcColourCellGroupDiff(itemDesc, groupDesc)
                    inGroup = 0
                    if i in group['members']:
                        groupIndex = key
                        groupDiff = diff
                        inGroup = 1
                    if diff < minDiff:
                        minDiff = diff
                        minIndex = key
                    minDiff = min(diff, minDiff)
                    diffsStr.append(' %3d%s' % (int(diff), '~' if inGroup else ','))
                diffsStr.append(' min=%3d(%3d) group=%3d(%3d)' % (minIndex, minDiff, groupIndex, groupDiff))
                if (groupIndex != -1) and minIndex != groupIndex:
                    diffsStr.append(' WRONG:%d' % (groupDiff - minDiff))
                    LOG(''.join(diffsStr), file=outputFile)
                    nWrong+=1
                #break ####################################################################
            LOG('COMP_CELL_TO_ALL_GROUPS colourCellGroup=%d colourCell=%d otherGroupColour=%d otherGroup=%d minDiff=%f minIndex=%f %s' % (colourCellsKey, i, colourGroupKey, key, minDiff, minIndex, '#' if colourCellsKey==colourGroupKey else ''), file=outputFile)
            #LOG('COMP_CELL_TO_GROUPS cells=%d groups=%d nWrong=%d nCells=%d %%=%f' % (colourCellsKey, colourGroupKey, nWrong, len(colourCells), nWrong/len(colourCells)), file=outputFile)

    for index1 in range(len(colourGroupKeys)):
        colourGroupKey1 = colourGroupKeys[index1]
        colourGroup1 = colourGroupDict[colourGroupKey1]
        for index2 in range(index1, len(colourGroupKeys)):
            colourGroupKey2 = colourGroupKeys[index2]
            colourGroup2 = colourGroupDict[colourGroupKey2]
            
            LOG('Compare group %d to %d :' % (colourGroupKey1, colourGroupKey2), file=outputFile)
            keys1 = list(colourGroup1.keys())
            keys2 = list(colourGroup2.keys())
            nKeys1 = len(keys1)
            nKeys2 = len(keys2)
            groupsStr = [' %4d,' % x for x in keys2[1:]]
            #LOG('   ' + ''.join(groupsStr), file=outputFile)
            #LOG('', file=outputFile)
            smallDiffs = []
            minDiff = 99999
            minKey1 = -1
            minKey2 = -1
            for i in range(0, nKeys1):
                diffsStr = []
                #diffsStr.append('%4d:' % keys[i] + '_' * (5 * i))
                diffsStr.append('%4d:' % keys1[i])
                for j in range(0, nKeys2):
                    if i == j and index1 == index2:
                        continue
                    descI = colourGroup1[keys1[i]]['desc']
                    descJ = colourGroup2[keys2[j]]['desc']
                    diff = calcColourGroupDiff(descI, descJ)
                    diffsStr.append(' %4d,' % diff)
                    if diff < 6:
                        smallDiffs.append((keys1[i], keys2[j], diff))
                    if diff < minDiff:
                        minDiff = diff
                        minKey1 = i
                        minKey2 = j
                #LOG(''.join(diffsStr), file=outputFile)
            #LOG('%d Cell groups are too similar:' % len(smallDiffs), file=outputFile)
            '''for i, j, diff in smallDiffs:
                LOG('Diff between %d and %d = %d' % (i, j, diff), file=outputFile)
                pprint(colourGroup1[i]['desc'], stream=outputFile)
                pprint(colourGroup2[j]['desc'], stream=outputFile)'''
            LOG('COMPARE_ALL_GROUPS colour1=%d colour2=%d minDiff=%f group1=%d group2=%d' % (index1, index2, minDiff, minKey1, minKey2), file=outputFile)
    
    return (nWrong, 0)  # nWrong, totalGroupError

def checkAllColourGroupsNEW(colourIdsToUse, outputFile):

    LOG('Compare item descriptions to groups for all colours', file=outputFile)
    absFunc = fabs
    nWrong = 0
    for colourCellsId in colourIdsToUse:
        colourCells = robotVisionUtils.loadPickledItem(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourCells%03d__.p' % colourCellsId))

        for colourGroupId in colourIdsToUse:
            colourGroups = robotVisionUtils.loadPickledItem(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourGroup%03d__.p' % colourGroupId))

            LOG('Comparing cells for colour %d to groups for %d' % (colourCellsId, colourGroupId), file=outputFile)

            for i in range(0, len(colourCells)):
                itemDesc = colourCells[i]
                diffsStr = ['Cell %d: ' % i]
                minDiff = 10000
                groupDiff = 10000
                minIndex = -1
                groupIndex = -1
                for j in range(len(colourGroups)):
                    group = colourGroups[j]
                    groupDesc = group['desc']
                    diff = calcColourCellGroupDiff(itemDesc, groupDesc)
                    inGroup = 0
                    if i in group['members']:
                        groupIndex = j
                        groupDiff = diff
                        inGroup = 1
                    if diff < minDiff:
                        minDiff = diff
                        minIndex = j
                    minDiff = min(diff, minDiff)
                    diffsStr.append(' %3d%s' % (int(diff), '~' if inGroup else ','))
                diffsStr.append(' min=%3d(%3d) group=%3d(%3d)' % (minIndex, minDiff, groupIndex, groupDiff))
                if (groupIndex != -1) and minIndex != groupIndex:
                    diffsStr.append(' WRONG:%d' % (groupDiff - minDiff))
                    LOG(''.join(diffsStr), file=outputFile)
                    nWrong+=1
            LOG('COMP_CELL_TO_ALL_GROUPS colourCellGroup=%d colourCell=%d otherGroupColour=%d otherGroup=%d minDiff=%f minIndex=%f %s' % (colourCellsId, i, colourGroupId, j, minDiff, minIndex, '#' if colourCellsId==colourGroupId else ''), file=outputFile)

    for colourIndex1 in range(len(colourIdsToUse)):
        colourGroupId1 = colourIdsToUse[colourIndex1]
        colourGroup1 = robotVisionUtils.loadPickledItem(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourGroup%03d__.p' % colourGroupId1))
        for colourIndex2 in range(colourIndex1, len(colourIdsToUse)):
            colourGroupId2 = colourIdsToUse[colourIndex2]
            colourGroup2 = robotVisionUtils.loadPickledItem(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourGroup%03d__.p' % colourGroupId2))
            
            LOG('Compare group %d to %d :' % (colourGroupId1, colourGroupId2), file=outputFile)
            smallDiffs = []
            minDiff = 99999
            minIndex1 = -1
            minIndex2 = -1
            for groupIndex1 in range(len(colourGroup1)):
                diffsStr = []
                diffsStr.append('%4d:' % groupIndex1)
                for groupIndex2 in range(len(colourGroup2)):
                    if groupIndex1 == groupIndex2 and colourIndex1 == colourIndex2:
                        continue
                    descI = colourGroup1[groupIndex1]['desc']
                    descJ = colourGroup2[groupIndex2]['desc']
                    diff = calcColourGroupDiff(descI, descJ)
                    diffsStr.append(' %4d,' % diff)
                    if diff < 6:
                        smallDiffs.append((groupIndex1, groupIndex2, diff))
                    if diff < minDiff:
                        minDiff = diff
                        minIndex1 = groupIndex1
                        minIndex2 = groupIndex2
            LOG('COMPARE_ALL_GROUPS colour1=%d colour2=%d minDiff=%f group1=%d group2=%d' % (groupIndex1, groupIndex2, minDiff, minIndex1, minIndex2), file=outputFile)
    
    return (nWrong, 0)  # nWrong, totalGroupError

def groupColours(dataDir, outputFile):
    nColours = robotVisionUtils.getNRobotRecTrainingImages(robotVisionUtils.ROBOT_REC_DATA_DIR)
    coloursToUse = robotVisionUtils.readColoursToUse(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, robotVisionUtils.ROBOT_REC_COLOURS_FILE))
    
    colourCellsDict = {}
    for index in range(0, nColours):
        if not index in coloursToUse:
            continue
        filename = r'%s\colourFeatures%03d.txt' % (robotVisionUtils.ROBOT_REC_DATA_DIR, index)
        if not os.path.isfile(filename):
            LOG('Cannot find colour features file for colour %d' % index)
            continue
        colourFile = open(filename)
        colourFileLines = [x for x in colourFile.readlines() if x != '']
        colourCells = calcColourFeats(colourFileLines, filename)
        colourCellsDict[index] = colourCells

    set_trace()
    collect()
    
    colourGroupDict = {}
    colours = sorted(colourCellsDict.keys())
    for colour in colours:
        LOG('Colour %d' % colour)
        colourGroupDict[colour] = calcColourGroups(colourCellsDict[colour], colour, outputFile)

        if colourGroupDict[colour]:
            checkColourGroups(colourGroupDict[colour], colourCellsDict[colour], colour, outputFile)
        else:
            LOG('Groups for colour %d empty' % colour, file=outputFile)
        
        collect()
    
    checkAllColourGroups(colourGroupDict, colourCellsDict, outputFile)
    
    return colourGroupDict


def groupColoursNEW(dataDir, outputFile):
    """
    Same as previous function, but avoiding memory errors
    """
    nColours = robotVisionUtils.getNRobotRecTrainingImages(robotVisionUtils.ROBOT_REC_DATA_DIR)
    coloursToUse = robotVisionUtils.readColoursToUse(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, robotVisionUtils.ROBOT_REC_COLOURS_FILE))
    
    # Calculate features for each colour cell from training images. Pickle instead of keeping
    # in memory to avoid memory errors
    cellsAlreadyCalcd = 0
    if not cellsAlreadyCalcd:
        for index in range(0, nColours):
            if not index in coloursToUse:
                continue
            filename = r'%s\colourFeatures%03d.txt' % (robotVisionUtils.ROBOT_REC_DATA_DIR, index)
            if not os.path.isfile(filename):
                LOG('Cannot find colour features file for colour %d' % index)
                continue
            colourFile = open(filename)
            colourFileLines = [x for x in colourFile.readlines() if x != '']
            colourCells = calcColourFeats(colourFileLines, filename)

            robotVisionUtils.pickleItem(colourCells, os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourCells%03d__.p' % index))
            collect()

    # Calculate set of group for each colour and pickle when done. We do not all groups in memory
    # as we may exceed 32-bit addressable memory
    colourGroupsAlreadyCalcd = 0
    if not colourGroupsAlreadyCalcd:
        for colour in coloursToUse:
            LOG('Colour %d' % colour)
            colourCells = robotVisionUtils.loadPickledItem(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourCells%03d__.p' % colour))
            colourGroups = calcColourGroups(colourCells, colour, outputFile)

            if colourGroups:
                checkColourGroups(colourGroups, colourCells, colour, outputFile)
                robotVisionUtils.pickleItem(colourGroups, os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourGroup%03d__.p' % colour))
            else:
                LOG('Groups for colour %d empty' % colour, file=outputFile)
            collect()
    
    colourGroupsAlreadyTested = 0
    if not colourGroupsAlreadyTested:
        (nWrong, totalGroupError) = checkAllColourGroupsNEW(coloursToUse, outputFile)
    
    # Save groups as a single dict so we can load for testing
    colourGroupDict = {}
    for colour in coloursToUse:
        colourGroups = robotVisionUtils.loadPickledItem(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, '__colourGroup%03d__.p' % colour))
        colourGroupDict[colour] = colourGroups
    
    return colourGroupDict

def writeColourGroups(colourGroups):
    robotVisionUtils.pickleItem(colourGroups, robotVisionUtils.COLOUR_GROUPS_PICKLE)

def writeColourAvgs(colourAvgs):
    robotVisionUtils.pickleItem(colourAvgs, robotVisionUtils.COLOUR_AVGS_PICKLE)
    
def writeColourGroupsCode(colourGroups, colourAvgs):
    modelFile = open('__robotRecParamsToCopyToCode__.txt', 'w')
    
    colourGroupsKeys = sorted(colourGroups.keys())
    colourAvgsKeys = sorted(colourAvgs.keys())
    assert colourGroupsKeys == colourAvgsKeys
    
    maxColourId = max(colourGroupsKeys)
    colourIndex = 0
    colourArray = []
    for colourArrayIndex in range(maxColourId + 1):
        thisColourIndex = -1
        if colourArrayIndex in colourGroupsKeys:
            thisColourIndex = colourIndex
            colourIndex += 1
        colourArray.append(thisColourIndex)
    
    LOG('#define N_ROBOT_COLOURS %d\n' % len(colourGroupsKeys), file=modelFile)
    LOG('#define MAX_COLOUR_ID %d\n' % maxColourId, file=modelFile)
    LOG('#define COLOUR_CONFIDENCE_THRESHOLD %f\n\n\n' % robotVisionUtils.COLOUR_TESTING_THRESHOLD, file=modelFile)
    
    LOG('const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS] = {', file=modelFile)
    for colourKey in colourGroupsKeys:
        LOG('%d,' % colourKey, file=modelFile)
    LOG('};\n\n', file=modelFile)

    LOG('const int RobotRecognitionModel_mapColourIdToIndex[MAX_COLOUR_ID + 1] = {', file=modelFile)
    for colourIndex in colourArray:
        LOG('%d,' % colourIndex, file=modelFile)
    LOG('};\n\n', file=modelFile)
    
    LOG('const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS] = {', file=modelFile)
    for colourKey in colourGroupsKeys:
        LOG('%12ff,' % colourAvgs[colourKey], file=modelFile)
    LOG('};\n\n', file=modelFile)
    
    for colourKey in colourGroupsKeys:
        colourRepresentation = colourGroups[colourKey]
        
        LOG('const int nGroupsForColour%d = %d;' % (colourKey, len(colourGroups[colourKey])), file=modelFile)
        LOG('const float groupsForColour%d[%d][6] = {' % (colourKey, len(colourGroups[colourKey])), file=modelFile)
        for colourGroupIndex in range(len(colourRepresentation)):
            colourGroup = colourRepresentation[colourGroupIndex]
            colourGroupDesc = colourGroup['desc']
            LOG('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // From %d cells, with group error %12f' % (colourGroupDesc[0], colourGroupDesc[1], colourGroupDesc[2], colourGroupDesc[3], colourGroupDesc[4], colourGroupDesc[5], len(colourGroup['members']), colourGroup['error']), file=modelFile)
        LOG('};\n', file=modelFile)

    LOG('\n\n\n\n\n\n\n', file=modelFile)
    LOG('\t\t\t// Get set of groups for this colour', file=modelFile)
    LOG('\t\t\tswitch (colourId)', file=modelFile)
    LOG('\t\t\t{', file=modelFile)
    
    for colourKey in colourGroupsKeys:
        LOG('\t\t\tcase %d:' % colourKey, file=modelFile)
        LOG('\t\t\t\tnGroups = nGroupsForColour%d;' % colourKey, file=modelFile)
        LOG('\t\t\t\tgroups = groupsForColour%d;' % colourKey, file=modelFile)
        LOG('\t\t\t\tbreak;', file=modelFile)

    LOG('\t\t\tdefault:', file=modelFile)
    LOG('\t\t\t\tnGroups = 0;', file=modelFile)
    LOG('\t\t\t\tgroups = 0;', file=modelFile)
    LOG('\t\t\t}', file=modelFile)
    
    modelFile.close()
    
    
    
    
    