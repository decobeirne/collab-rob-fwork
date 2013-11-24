import sys
import os.path
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir))
import math
import gc
import time
import pprint
from pdb import set_trace
import robotVisionUtils
import utils

LOG = utils.log
WRAP_FUNCTION = utils.wrapFunction
HAVE_CUSTOM_MODULE = 0  # Started working on updating this - but realised it will just be quicker to go with python and fix up later

def compareCells(desc1, desc2):
    absFunc = math.fabs
    diff = absFunc(desc1[0] - desc2[0]) +  \
           absFunc(desc1[1] - desc2[1]) +  \
           absFunc(desc1[2] - desc2[2]) +  \
           absFunc(desc1[3] - desc2[3]) +  \
           absFunc(desc1[4] - desc2[4]) +  \
           absFunc(desc1[5] - desc2[5]);
    return diff

def compareCellsNEW(desc1, desc2, cellStdDevs):
    absFunc = math.fabs
    diff = absFunc(desc1[0] - desc2[0]) / cellStdDevs[0] + \
           absFunc(desc1[1] - desc2[1]) / cellStdDevs[1] + \
           absFunc(desc1[2] - desc2[2]) / cellStdDevs[2] + \
           absFunc(desc1[3] - desc2[3]) / cellStdDevs[3] + \
           absFunc(desc1[4] - desc2[4]) / cellStdDevs[4] + \
           absFunc(desc1[5] - desc2[5]) / cellStdDevs[5];
    return diff

def calculateCellGroupDescription(groupMembers, cellDescs, refFeats):
    groupDesc = [0] * 6
    for cellIndex in groupMembers:
        cellDesc = cellDescs[cellIndex]
        for i in range(0, 6):
            groupDesc[i] += cellDesc[i][refFeats[i]]
    nGroupMembersInv = float(1) / len(groupMembers)
    desc = [x * nGroupMembersInv for x in groupDesc]
    return desc

def calculateCellGroupDescriptionNEW(groupMembers, cellDescs):
    groupDesc = [0] * 6
    for cellIndex in groupMembers:
        cellDesc = cellDescs[cellIndex]
        groupDesc[0] += cellDesc[0]
        groupDesc[1] += cellDesc[1]
        groupDesc[2] += cellDesc[2]
        groupDesc[3] += cellDesc[3]
        groupDesc[4] += cellDesc[4]
        groupDesc[5] += cellDesc[5]
    nGroupMembersInv = float(1) / len(groupMembers)
    desc = [x * nGroupMembersInv for x in groupDesc]
    return desc

def calculateCellGroupError(groupMembers, groupDesc, cellDescs):
    groupError = 0
    absFunc = math.fabs
    for groupMember in groupMembers:
        member = cellDescs[groupMember]
        diff = absFunc(member[0] - groupDesc[0]) + \
               absFunc(member[1] - groupDesc[1]) + \
               absFunc(member[2] - groupDesc[2]) + \
               absFunc(member[3] - groupDesc[3]) + \
               absFunc(member[4] - groupDesc[4]) + \
               absFunc(member[5] - groupDesc[5]);
        groupError += diff

    #groupError *= (2 / len(groupMembers))
    return groupError

def compareCellDescs(desc1, desc2, refFeats):
    def compareCellFeats(feat1, feat2, refFeat):
        return math.fabs(feat1[refFeat] - feat2[refFeat])

    featTuples = zip(desc1, desc2, refFeats)
    diffs = (compareCellFeats(x[0], x[1], x[2]) for x in featTuples)
    diff = sum(diffs)
    return diff

@WRAP_FUNCTION
def calcCellGroupsNEW2(descs, outputFile):
    '''
    Create groups of cell descriptions.
    
    Input is a set of cells, all either occupied or not. Cell features are stated as a
    ratio of the corresponding image features.
    
    Each cell desc is a list, containing a description for each appearance feature. Each
    feature is a list of ratios of the actual feature for that cell against features for
    its original image.
    
    Ref feats is a list of 6 integers. The first value corresponds to the index of the image
    feature that best describes the first cell feature, i.e. the image feature with least 
    variance relative to that cell feature over the grid of cells.
    '''
    # For each cellDesc, calculate simularity to each other celDesc. Store simularity
    # measures in order list.
    startTimeTotal = time.clock()
    nCells = len(descs)
    diffDicts = {}
    absFunc = math.fabs
    for i in range(0, nCells):
        diffsThisCell = {}
        cellDesc = descs[i]
        for j in range(i + 1, nCells):
            # This is being called n! times. Avoid overhead of calling a function
            otherDesc = descs[j]
            diff = absFunc(cellDesc[0] - otherDesc[0]) + \
                   absFunc(cellDesc[1] - otherDesc[1]) + \
                   absFunc(cellDesc[2] - otherDesc[2]) + \
                   absFunc(cellDesc[3] - otherDesc[3]) + \
                   absFunc(cellDesc[4] - otherDesc[4]) + \
                   absFunc(cellDesc[5] - otherDesc[5]);
            # Tuple contains: (diff, groupError, (index, type), (index, type)), where type={0:singleItem, 1=group}
            otherKey = 10000 + j  # 10000=item 20000=group, use integer arithmetic to get type from key
            diffsThisCell[otherKey] = (diff, 0)
        thisKey = 10000 + i
        diffDicts[thisKey] = diffsThisCell
        if diffsThisCell:
            closestKey = min(diffsThisCell.items(), key=lambda x: (x[1][0] + x[1][1]))[0]  # items() will return tuples of the form (key, value), so take item [0]
            closestValue = diffsThisCell[closestKey][0] + diffsThisCell[closestKey][1]
            diffsThisCell['closestMatch'] = (99999, 99999, closestValue, closestKey, thisKey)  # We only access items 0 and 1 for regular tuples
        else:
            # Just add this to avoid having to check for exceptions in the loop below; last cell will have no diffs
            diffsThisCell['closestMatch'] = (99999, 99999, 99999, -1, thisKey)

    # Group cell appearances together
    groups = {}
    nextGroupId = 0
    itemsNotYetGrouped = [x for x in range(len(descs))]
    calcGroupDescFunc = calculateCellGroupDescriptionNEW
    calcGroupErrorFunc = calculateCellGroupError
    #while itemsNotYetGrouped and (len(groups) + len(itemsNotYetGrouped)) > robotVisionUtils.N_CELL_GROUPS_MAX:
    while (len(groups) + len(itemsNotYetGrouped)) > robotVisionUtils.N_CELL_GROUPS_MAX:
    
        # Re-calc which entity has the best match to another entity
        closestMatch = min((diffDicts[x]['closestMatch'] for x in diffDicts), key=lambda x: x[2])
        LOG('(%d left) Creating group %3d from diff: %s' % (len(itemsNotYetGrouped), nextGroupId, repr(closestMatch[2:])), file=outputFile)
        keyI = closestMatch[4]
        keyJ = closestMatch[3]
        typeI = keyI // 10000  # 10000=item 20000=group
        typeJ = keyJ // 10000
        idI = keyI % 10000
        idJ = keyJ % 10000
        
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
        newKey = 20000 + nextGroupId
        groups[nextGroupId] = newGroup
        nextGroupId += 1
        newGroup['desc'] = calcGroupDescFunc(newGroup['members'], descs)
        newGroup['error'] = calcGroupErrorFunc(newGroup['members'], newGroup['desc'], descs)
        
        # Create new dictDiff
        newDictDiff = {}
        for otherKey in diffDicts.keys():
            otherType = otherKey // 10000
            otherId = otherKey % 10000
            if otherType == 2:
                descToCompTo = groups[otherId]['desc']
                newError = newGroup['error'] + groups[otherId]['error']
            else:
                descToCompTo = descs[otherId]
                newError = newGroup['error']
            newDiff = absFunc(newGroup['desc'][0] - descToCompTo[0]) + \
                      absFunc(newGroup['desc'][1] - descToCompTo[1]) + \
                      absFunc(newGroup['desc'][2] - descToCompTo[2]) + \
                      absFunc(newGroup['desc'][3] - descToCompTo[3]) + \
                      absFunc(newGroup['desc'][4] - descToCompTo[4]) + \
                      absFunc(newGroup['desc'][5] - descToCompTo[5]);
            newDictDiff[otherKey] = (newDiff, newError)
        closestKey = min(newDictDiff.items(), key=lambda x: (x[1][0] + x[1][1]))[0]
        closestValue = newDictDiff[closestKey][0] + newDictDiff[closestKey][1]
        newDictDiff['closestMatch'] = (99999, 99999, closestValue, closestKey, newKey)
        diffDicts[newKey] = newDictDiff

    # Put any remaining items into groups on their own
    LOG('Items not joined to any groups:', file=outputFile)
    pprint.pprint(itemsNotYetGrouped, stream=outputFile)
    for id in itemsNotYetGrouped:
        newGroup = {'members': [id], 'desc': descs[id], 'error':0}
        newGroup['id'] = nextGroupId
        groups[nextGroupId] = newGroup
        nextGroupId += 1

    LOG('%d groups:' % len(groups), file=outputFile)
    for groupKey in groups:
        group = groups[groupKey]
        LOG('Group %d (%d)' % (group['id'], len(group['members'])), file=outputFile)
        LOG(repr(group['members']), file=outputFile)
        group.pop('id')

    elapsedTime = time.clock() - startTimeTotal
    LOG('TIME TOTAL %f' % elapsedTime, outputFile)

    return groups

@WRAP_FUNCTION
def calcCellGroupsNEW3(descs, cellStdDevs, outputFile):
    '''
    Create groups of cell descriptions.
    
    Input is a set of cells, all either occupied or not. Cell features are stated as a
    ratio of the corresponding image features.
    
    Each cell desc is a list, containing a description for each appearance feature. Each
    feature is a list of ratios of the actual feature for that cell against features for
    its original image.
    
    Ref feats is a list of 6 integers. The first value corresponds to the index of the image
    feature that best describes the first cell feature, i.e. the image feature with least 
    variance relative to that cell feature over the grid of cells.
    '''
    # For each cellDesc , calculate simularity to each other celDesc. Store simularity
    # measures in order list.
    startTimeTotal = time.clock()
    nCells = len(descs)
    diffDicts = {}
    absFunc = math.fabs
    for i in range(0, nCells):
        diffsThisCell = {}
        cellDesc = descs[i]
        for j in range(i + 1, nCells):
            # This is being called n! times. Avoid overhead of calling a function
            otherDesc = descs[j]
            diff = absFunc(cellDesc[0] - otherDesc[0]) / cellStdDevs[0] + \
                   absFunc(cellDesc[1] - otherDesc[1]) / cellStdDevs[1] + \
                   absFunc(cellDesc[2] - otherDesc[2]) / cellStdDevs[2] + \
                   absFunc(cellDesc[3] - otherDesc[3]) / cellStdDevs[3] + \
                   absFunc(cellDesc[4] - otherDesc[4]) / cellStdDevs[4] + \
                   absFunc(cellDesc[5] - otherDesc[5]) / cellStdDevs[5];
            # Tuple contains: (diff, groupError, (index, type), (index, type)), where type={0:singleItem, 1=group}
            otherKey = 10000 + j  # 10000=item 20000=group, use integer arithmetic to get type from key
            diffsThisCell[otherKey] = (diff, 0)
        thisKey = 10000 + i
        diffDicts[thisKey] = diffsThisCell
        if diffsThisCell:
            closestKey = min(diffsThisCell.items(), key=lambda x: (x[1][0] + x[1][1]))[0]  # items() will return tuples of the form (key, value), so take item [0]
            closestValue = diffsThisCell[closestKey][0] + diffsThisCell[closestKey][1]
            diffsThisCell['closestMatch'] = (99999, 99999, closestValue, closestKey, thisKey)  # We only access items 0 and 1 for regular tuples
        else:
            # Just add this to avoid having to check for exceptions in the loop below; last cell will have no diffs
            diffsThisCell['closestMatch'] = (99999, 99999, 99999, -1, thisKey)

    # Group cell appearances together
    groups = {}
    nextGroupId = 0
    itemsNotYetGrouped = [x for x in range(len(descs))]
    calcGroupDescFunc = calculateCellGroupDescriptionNEW
    calcGroupErrorFunc = calculateCellGroupError
    #while itemsNotYetGrouped and (len(groups) + len(itemsNotYetGrouped)) > robotVisionUtils.N_CELL_GROUPS_MAX:
    while (len(groups) + len(itemsNotYetGrouped)) > robotVisionUtils.N_CELL_GROUPS_MAX:
    
        # Re-calc which entity has the best match to another entity
        closestMatch = min((diffDicts[x]['closestMatch'] for x in diffDicts), key=lambda x: x[2])
        LOG('(%d left) Creating group %3d from diff: %s' % (len(itemsNotYetGrouped), nextGroupId, repr(closestMatch[2:])), file=outputFile)
        keyI = closestMatch[4]
        keyJ = closestMatch[3]
        typeI = keyI // 10000  # 10000=item 20000=group
        typeJ = keyJ // 10000
        idI = keyI % 10000
        idJ = keyJ % 10000
        
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
        newKey = 20000 + nextGroupId
        groups[nextGroupId] = newGroup
        nextGroupId += 1
        newGroup['desc'] = calcGroupDescFunc(newGroup['members'], descs)
        newGroup['error'] = calcGroupErrorFunc(newGroup['members'], newGroup['desc'], descs)
        
        # Create new dictDiff
        newDictDiff = {}
        for otherKey in diffDicts.keys():
            otherType = otherKey // 10000
            otherId = otherKey % 10000
            if otherType == 2:
                descToCompTo = groups[otherId]['desc']
                newError = newGroup['error'] + groups[otherId]['error']
            else:
                descToCompTo = descs[otherId]
                newError = newGroup['error']
            newDiff = absFunc(newGroup['desc'][0] - descToCompTo[0]) / cellStdDevs[0] + \
                      absFunc(newGroup['desc'][1] - descToCompTo[1]) / cellStdDevs[1] + \
                      absFunc(newGroup['desc'][2] - descToCompTo[2]) / cellStdDevs[2] + \
                      absFunc(newGroup['desc'][3] - descToCompTo[3]) / cellStdDevs[3] + \
                      absFunc(newGroup['desc'][4] - descToCompTo[4]) / cellStdDevs[4] + \
                      absFunc(newGroup['desc'][5] - descToCompTo[5]) / cellStdDevs[5];
            newDictDiff[otherKey] = (newDiff, newError)
        closestKey = min(newDictDiff.items(), key=lambda x: (x[1][0] + x[1][1]))[0]
        closestValue = newDictDiff[closestKey][0] + newDictDiff[closestKey][1]
        newDictDiff['closestMatch'] = (99999, 99999, closestValue, closestKey, newKey)
        diffDicts[newKey] = newDictDiff

    # Put any remaining items into groups on their own
    LOG('Items not joined to any groups:', file=outputFile)
    pprint.pprint(itemsNotYetGrouped, stream=outputFile)
    for id in itemsNotYetGrouped:
        newGroup = {'members': [id], 'desc': descs[id], 'error':0}
        newGroup['id'] = nextGroupId
        groups[nextGroupId] = newGroup
        nextGroupId += 1

    LOG('%d groups:' % len(groups), file=outputFile)
    for groupKey in groups:
        group = groups[groupKey]
        LOG('Group %d (%d)' % (group['id'], len(group['members'])), file=outputFile)
        LOG(repr(group['members']), file=outputFile)
        group.pop('id')

    elapsedTime = time.clock() - startTimeTotal
    LOG('TIME TOTAL %f' % elapsedTime, outputFile)

    return groups

@WRAP_FUNCTION
def checkCellGroups(cellGroups, cellDescs, outputFile):
    '''
    Sanity check to verify that the groups make sense
    '''
    if not cellDescs:
        LOG('No cells to check', file=outputFile)
        return 0, 0

    LOG('Compare cell descs to groups', file=outputFile)
    absFunc = math.fabs
    nWrong = 0
    for i in range(0, len(cellDescs)):
        cellDesc = cellDescs[i]
        diffsStr = ['Cell %d: ' % i]
        minDiff = 10000
        groupDiff = 10000
        minIndex = -1
        groupIndex = -1
        for key, cellGroup in cellGroups.items():
            groupDesc = cellGroup['desc']
            diff = absFunc(cellDesc[0] - groupDesc[0]) + \
                   absFunc(cellDesc[1] - groupDesc[1]) + \
                   absFunc(cellDesc[2] - groupDesc[2]) + \
                   absFunc(cellDesc[3] - groupDesc[3]) + \
                   absFunc(cellDesc[4] - groupDesc[4]) + \
                   absFunc(cellDesc[5] - groupDesc[5]);
            inGroup = 0
            if i in cellGroup['members']:
                groupIndex = key
                groupDiff = diff
                inGroup = 1
            if diff < minDiff:
                minDiff = diff
                minIndex = key
            minDiff = min(diff, minDiff)
            diffsStr.append(' %2d%s' % (int(diff), '~' if inGroup else ','))
        diffsStr.append(' min=%2d(%2d) group=%2d(%2d)' % (minIndex, minDiff, groupIndex, groupDiff))
        if minIndex != groupIndex:
            diffsStr.append(' WRONG:%d' % (groupDiff - minDiff))
            LOG(''.join(diffsStr), file=outputFile)
            nWrong+=1
    LOG('COMP_CELL_TO_GROUPS nWrong=%d nCells=%d %%=%f' % (nWrong, len(cellDescs), nWrong/len(cellDescs)), file=outputFile)

    LOG('Cell group errors', file=outputFile)
    groupErrorList = [cellGroups[x]['error'] for x in cellGroups]
    totalGroupError = sum(groupErrorList)
    LOG(repr(groupErrorList), file=outputFile)
    LOG('CELL_GROUP_ERROR sum=%f avg=%f' % (totalGroupError, sum(groupErrorList) / len(groupErrorList)), file=outputFile)
    
    LOG('Compare cell groups to eachother:', file=outputFile)
    keys = list(cellGroups.keys())
    nKeys = len(keys)
    groupsStr = [' %2d,' % x for x in keys[1:]]
    LOG('   ' + ''.join(groupsStr), file=outputFile)
    LOG('', file=outputFile)
    smallDiffs = []
    for i in range(0, nKeys):
        diffsStr = []
        diffsStr.append('%2d:' % keys[i] + '_' * (4 * i))
        for j in range(i + 1, nKeys):
            descI = cellGroups[keys[i]]['desc']
            descJ = cellGroups[keys[j]]['desc']
            diff = absFunc(descI[0] - descJ[0]) + \
                   absFunc(descI[1] - descJ[1]) + \
                   absFunc(descI[2] - descJ[2]) + \
                   absFunc(descI[3] - descJ[3]) + \
                   absFunc(descI[4] - descJ[4]) + \
                   absFunc(descI[5] - descJ[5]);
            diffsStr.append(' %2d,' % diff)
            if diff < 6:
                smallDiffs.append((keys[i], keys[j], diff))
        LOG(''.join(diffsStr), file=outputFile)
    LOG('%d Cell groups are too similar:' % len(smallDiffs), file=outputFile)
    for i, j, diff in smallDiffs:
        LOG('Diff between %d and %d = %d' % (i, j, diff), file=outputFile)
        pprint.pprint(cellGroups[i]['desc'], stream=outputFile)
        pprint.pprint(cellGroups[j]['desc'], stream=outputFile)
    
    return (nWrong, totalGroupError)

@WRAP_FUNCTION
def checkCellGroupsNEW(cellGroups, cellDescs, cellStdDevs, outputFile):
    '''
    Sanity check to verify that the groups make sense
    '''
    if not cellDescs:
        LOG('No cells to check', file=outputFile)
        return 0, 0

    LOG('Compare cell descs to groups', file=outputFile)
    absFunc = math.fabs
    nWrong = 0
    for i in range(0, len(cellDescs)):
        cellDesc = cellDescs[i]
        diffsStr = ['Cell %d: ' % i]
        minDiff = 10000
        groupDiff = 10000
        minIndex = -1
        groupIndex = -1
        for key, cellGroup in cellGroups.items():
            groupDesc = cellGroup['desc']
            diff = absFunc(cellDesc[0] - groupDesc[0]) / cellStdDevs[0] + \
                   absFunc(cellDesc[1] - groupDesc[1]) / cellStdDevs[1] + \
                   absFunc(cellDesc[2] - groupDesc[2]) / cellStdDevs[2] + \
                   absFunc(cellDesc[3] - groupDesc[3]) / cellStdDevs[3] + \
                   absFunc(cellDesc[4] - groupDesc[4]) / cellStdDevs[4] + \
                   absFunc(cellDesc[5] - groupDesc[5]) / cellStdDevs[5];
            inGroup = 0
            if i in cellGroup['members']:
                groupIndex = key
                groupDiff = diff
                inGroup = 1
            if diff < minDiff:
                minDiff = diff
                minIndex = key
            minDiff = min(diff, minDiff)
            diffsStr.append(' %2d%s' % (int(diff), '~' if inGroup else ','))
        diffsStr.append(' min=%2d(%2d) group=%2d(%2d)' % (minIndex, minDiff, groupIndex, groupDiff))
        if minIndex != groupIndex:
            diffsStr.append(' WRONG:%d' % (groupDiff - minDiff))
            LOG(''.join(diffsStr), file=outputFile)
            nWrong+=1
    LOG('COMP_CELL_TO_GROUPS nWrong=%d nCells=%d %%=%f' % (nWrong, len(cellDescs), nWrong/len(cellDescs)), file=outputFile)

    LOG('Cell group errors', file=outputFile)
    groupErrorList = [cellGroups[x]['error'] for x in cellGroups]
    totalGroupError = sum(groupErrorList)
    LOG(repr(groupErrorList), file=outputFile)
    LOG('CELL_GROUP_ERROR sum=%f avg=%f' % (totalGroupError, sum(groupErrorList) / len(groupErrorList)), file=outputFile)
    
    LOG('Compare cell groups to eachother:', file=outputFile)
    keys = list(cellGroups.keys())
    nKeys = len(keys)
    groupsStr = [' %2d,' % x for x in keys[1:]]
    LOG('   ' + ''.join(groupsStr), file=outputFile)
    LOG('', file=outputFile)
    smallDiffs = []

    for i in range(0, nKeys):
        diffsStr = []
        diffsStr.append('%2d:' % keys[i] + '_' * (4 * i))
        for j in range(i + 1, nKeys):
            descI = cellGroups[keys[i]]['desc']
            descJ = cellGroups[keys[j]]['desc']
            diff = absFunc(descI[0] - descJ[0]) / cellStdDevs[0] + \
                   absFunc(descI[1] - descJ[1]) / cellStdDevs[1] + \
                   absFunc(descI[2] - descJ[2]) / cellStdDevs[2] + \
                   absFunc(descI[3] - descJ[3]) / cellStdDevs[3] + \
                   absFunc(descI[4] - descJ[4]) / cellStdDevs[4] + \
                   absFunc(descI[5] - descJ[5]) / cellStdDevs[5];
            diffsStr.append(' %2d,' % diff)
            if diff < 6:
                smallDiffs.append((keys[i], keys[j], diff))
        LOG(''.join(diffsStr), file=outputFile)
    LOG('%d Cell groups are too similar:' % len(smallDiffs), file=outputFile)
    for i, j, diff in smallDiffs:
        LOG('Diff between %d and %d = %d' % (i, j, diff), file=outputFile)
        pprint.pprint(cellGroups[i]['desc'], stream=outputFile)
        pprint.pprint(cellGroups[j]['desc'], stream=outputFile)
    
    return (nWrong, totalGroupError)

@WRAP_FUNCTION
def calculateCellDescriptionsNEW(imageGroup, outputFile):
    '''
    For each image in given image group, calculate ratios of cellFeatures to
    imgFeatures. These values are read from files.
    '''
    tempList = [0] * 6
    unoccupiedDescs = []
    occupiedDescs = []
    cellLists = (unoccupiedDescs, occupiedDescs)
    for imageIndex in imageGroup['members']:
        # Read cell and (inverse) image descriptions from files
        cellFeatsFilename = r'%s\cellFeatures%03d.txt' % (robotVisionUtils.OBST_REC_DATA_DIR, imageIndex)
        imgInvFeatsFilename = r'%s\imgInvFeatures%03d.txt' % (robotVisionUtils.OBST_REC_DATA_DIR, imageIndex)
    
        cellFeatsFile = open(cellFeatsFilename)
        imgInvFeatsFile = open(imgInvFeatsFilename)
        
        cellFeatsRawList = cellFeatsFile.readlines()
        imgInvFeatsRawList = imgInvFeatsFile.readlines()
        
        cellFeatsList = [y for y in (x.strip() for x in cellFeatsRawList) if y != '']
        imgInvFeatsList = [float(y) for y in (x.strip() for x in imgInvFeatsRawList) if y != '']

        nCellsThisImage = robotVisionUtils.OCCUP_GRID_X_DIMS * int(cellFeatsList[0]) # Read occupancy grid Y dimensions from start of list

        for cellIndex in range(0, nCellsThisImage):
            listIndex = 1 + cellIndex * 7

            isOccupied = int(cellFeatsList[listIndex])
            tempList[0] = float(cellFeatsList[listIndex+1])
            tempList[1] = float(cellFeatsList[listIndex+2])
            tempList[2] = float(cellFeatsList[listIndex+3])
            tempList[3] = float(cellFeatsList[listIndex+4])
            tempList[4] = float(cellFeatsList[listIndex+5])
            tempList[5] = float(cellFeatsList[listIndex+6])
            
            if isOccupied == 2: # Cell had been marked as unknown in the training data
                continue
            
            cellDesc = list(tempList)
            cellLists[isOccupied].append(cellDesc)

    LOG('Group cells: nUnoccup %d nOccup %d' % (len(unoccupiedDescs), len(occupiedDescs)), file=outputFile)

    # Each cell of each image in this group has now been added to either the unoccupied or occupied list.
    return unoccupiedDescs, occupiedDescs

def calculateCellGroupFeatStdDevs(dataDir, outputFile):
    cellList = []

    cellFeatsFiles = [x for x in os.listdir(dataDir) if 'cellFeatures' in x]
    for cellFeatsFile in cellFeatsFiles:
        cellFeatsRawList = open(os.path.join(dataDir, cellFeatsFile)).readlines()
        cellFeatsList = [y for y in (x.strip() for x in cellFeatsRawList) if y != '']
        nCellsThisImage = robotVisionUtils.OCCUP_GRID_X_DIMS * int(cellFeatsList[0]) # Read occupancy grid Y dimensions from start of list
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
            
            if isOccupied == 2: # Cell had been marked as unknown in the training data
                continue

            cellList.append(cellDesc)
    
    nCells = len(cellList)
    
    avgs = [0] * 6
    for cellDesc in cellList:
        avgs[0] += cellDesc[0]
        avgs[1] += cellDesc[1]
        avgs[2] += cellDesc[2]
        avgs[3] += cellDesc[3]
        avgs[4] += cellDesc[4]
        avgs[5] += cellDesc[5]
    
    avgs[0] /= nCells
    avgs[1] /= nCells
    avgs[2] /= nCells
    avgs[3] /= nCells
    avgs[4] /= nCells
    avgs[5] /= nCells
    
    stdDevs = [0] * 6
    absFunc = math.fabs
    for cellDesc in cellList:
        stdDevs[0] += absFunc(cellDesc[0] - avgs[0])
        stdDevs[1] += absFunc(cellDesc[1] - avgs[1])
        stdDevs[2] += absFunc(cellDesc[2] - avgs[2])
        stdDevs[3] += absFunc(cellDesc[3] - avgs[3])
        stdDevs[4] += absFunc(cellDesc[4] - avgs[4])
        stdDevs[5] += absFunc(cellDesc[5] - avgs[5])

    stdDevs[0] /= nCells
    stdDevs[1] /= nCells
    stdDevs[2] /= nCells
    stdDevs[3] /= nCells
    stdDevs[4] /= nCells
    stdDevs[5] /= nCells

    return stdDevs

@WRAP_FUNCTION
def groupCells(imageGroups, imageStdDevs, imageDescs, outputFile):
    '''
    Have a set of image groups each with associated description. For each group, also
    have set of descriptions of unoccupied and occupied cells.
    
    As there may be multiple distinct appearances for occupied/unoccupied cells, cells
    are grouped together in groups labeled as occupied/unoccupied.
    '''
    if HAVE_CUSTOM_MODULE:
        import imageUtils
        mergeFunction = imageUtils.groupCells
    else:
        mergeFunction = calcCellGroupsNEW2
    
    #cellStdDevs = calculateCellGroupFeatStdDevs(robotVisionUtils.OBST_REC_DATA_DIR, outputFile)

    index = 0
    nWrong = 0
    groupError = 0
    for key, group in imageGroups.items():
        LOG('Generating cell groups for %d images in group %d: %d' % (len(group['members']), index, key), file=outputFile)

        unoccupiedDescs, occupiedDescs = calculateCellDescriptionsNEW(group, outputFile)

        LOG('Creating unoccupied groups:', file=outputFile)
        if not unoccupiedDescs:
            group['unoccupiedGroups'] = {}
        else:
            group['unoccupiedGroups'] = mergeFunction(unoccupiedDescs, outputFile)

        LOG('Creating occupied groups:', file=outputFile)
        if not occupiedDescs:
            group['occupiedGroups'] = {}
        else:
            group['occupiedGroups'] = mergeFunction(occupiedDescs, outputFile)

        LOG('Checking unoccupied groups:', file=outputFile)
        (nWrongThisImage, groupErrorThisImage) = checkCellGroups(group['unoccupiedGroups'], unoccupiedDescs, outputFile)
        nWrong += nWrongThisImage
        groupError += groupErrorThisImage

        LOG('Checking occupied groups:', file=outputFile)
        (nWrongThisImage, groupErrorThisImage) = checkCellGroups(group['occupiedGroups'], occupiedDescs, outputFile)
        nWrong += nWrongThisImage
        groupError += groupErrorThisImage
        
        gc.collect()

        index += 1

    LOG('COMP_CELL_TO_GROUPS_TOTAL nWrong=%d' % nWrong, file=outputFile)
    LOG('TOTAL_CELL_GROUP_ERROR nWrong=%d' % groupError, file=outputFile)

    #return cellStdDevs
