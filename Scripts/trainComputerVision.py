
import utils
import os, re, math, time, pickle, sys
import pprint, pdb


nImages = 0
occupGridXDims = 19
nImageGroupsMax = 32
nCellGroupsMax = 32
nCellsBlock = 600 # 1000
nCellsSplitIntoBlocks = nCellsBlock * 1.5
onLaptop = 1
if onLaptop:
    rootDir = r'D:\thesis'
else:
    rootDir = r'C:\stuff\t'
coopLocDir = os.path.normpath(os.path.join(rootDir, r'calibration\cooperativeLocalisation'))
obstDetDir = os.path.normpath(os.path.join(rootDir, r'calibration\obstacleDetection'))
log = utils.log
wrapFunction = utils.wrapFunction


################################################################################################
#
# Utils
#
################################################################################################


def __propogateMergedMembers(group):
    '''
    If a group has been merged onto another, then remove its members and append them onto the parent group.
    '''
    if 'dest' in group.keys():
        destGroup = group['dest']
        destGroup['members'].extend(group['members'])
        group['members'] = []
        __propogateMergedMembers(destGroup)

def __findEndGroup(group):
    if not 'dest' in group.keys():
        return group
    endGroup = group['dest']
    return __findEndGroup(endGroup)


################################################################################################
#
# Robot groups
#
################################################################################################


def calculateRobotGroups_0():
    '''
    Calculate descriptions for cellGroups corresponding to robots.
    '''
    files = [x for x in os.listdir(coopLocDir) if x.startswith('cellFeatures')]
    if not files:
        print('ERROR: couldn\'t find any files in %s' % coopLocDir)
        return -1

    nCoopLocImages = int(files[-1][12:15])
    colourDicts = {}
    for imageIndex in range(nCoopLocImages):        
        cellFeatsFilename = r'%s\cellFeatures%03d.txt' % (coopLocDir, imageIndex)        
        imgInvFeatsFilename = r'%s\imgInvFeatures%03d.txt' % (coopLocDir, imageIndex)

        cellFeatsFile = open(cellFeatsFilename)
        cellFeatsList = cellFeatsFile.readlines()
        cellFeatsList = [y for y in (x.strip() for x in cellFeatsList) if y != '']

        imgInvFeatsFile = open(imgInvFeatsFilename)
        imgInvFeatsList = imgInvFeatsFile.readlines()
        imgInvFeatsList = [float(y) for y in (x.strip() for x in imgInvFeatsList) if y != '']

        nItemsAtStartOfCellFeats = 1
        nCellsThisImage = occupGridXDims * int(cellFeatsList[0]) # Read occupancy grid Y dimensions from start of list
        for cellIndex in range(nCellsThisImage):
            listIndex = nItemsAtStartOfCellFeats + cellIndex * 7
            category = int(cellFeatsList[listIndex])
            if category in (18, 19, 20):  # (white card, cannot est, background)
                continue
            if category not in colourDicts:
                colourDicts[category] = {'cellList':[]}
            cellDesc = [0.0] * 6
            cellDesc[0] = float(cellFeatsList[listIndex+1]) * imgInvFeatsList[2]  # V (brightness)
            cellDesc[1] = float(cellFeatsList[listIndex+2]) * imgInvFeatsList[2]
            cellDesc[2] = float(cellFeatsList[listIndex+3]) * imgInvFeatsList[2]
            cellDesc[3] = float(cellFeatsList[listIndex+4]) * imgInvFeatsList[2]
            cellDesc[4] = float(cellFeatsList[listIndex+5]) * imgInvFeatsList[2]
            cellDesc[5] = float(cellFeatsList[listIndex+6]) * imgInvFeatsList[2]
            colourDicts[category]['cellList'].append(cellDesc)

    for colour, colourDict in colourDicts.items():
        nCells = len(colourDict['cellList'])
        groupDesc = [0.0] * 6
        for cellDesc in colourDict['cellList']:
            for i in range(6):
                groupDesc[i] += cellDesc[i]
        nCellsInv = 1.0 / nCells
        groupDesc = [x * nCellsInv for x in groupDesc]
        colourDict['desc'] = groupDesc
        print('Description of colour %d from %d cells...' % (colour, nCells))
        pprint.pprint(groupDesc)

        groupDescVar = [0.0] * 6
        for cellDesc in colourDict['cellList']:
            for i in range(6):
                groupDescVar[i] += math.fabs(cellDesc[i] - groupDesc[i])
        groupDescVar = [x * nCellsInv for x in groupDescVar]
        colourDict['desc'].extend(groupDescVar)
        print('Variance in features...')
        pprint.pprint(groupDescVar)
    return colourDicts


def calculateRobotGroups():
    '''
    Calculate descriptions for cellGroups corresponding to robots.
    '''
    files = [x for x in os.listdir(coopLocDir) if x.startswith('cellFeatures')]
    if not files:
        print('ERROR: couldn\'t find any files in %s' % coopLocDir)
        return -1

    # Gather cell descs into dict.
    nCoopLocImages = int(files[-1][12:15])
    colourDicts = {}
    for imageIndex in range(nCoopLocImages):        
        cellFeatsFilename = r'%s\cellFeatures%03d.txt' % (coopLocDir, imageIndex)        
        imgInvFeatsFilename = r'%s\imgInvFeatures%03d.txt' % (coopLocDir, imageIndex)

        cellFeatsFile = open(cellFeatsFilename)
        cellFeatsList = cellFeatsFile.readlines()
        cellFeatsList = [y for y in (x.strip() for x in cellFeatsList) if y != '']

        imgInvFeatsFile = open(imgInvFeatsFilename)
        imgInvFeatsList = imgInvFeatsFile.readlines()
        imgInvFeatsList = [float(y) for y in (x.strip() for x in imgInvFeatsList) if y != '']

        nItemsAtStartOfCellFeats = 1
        nCellsThisImage = occupGridXDims * int(cellFeatsList[0]) # Read occupancy grid Y dimensions from start of list
        for cellIndex in range(nCellsThisImage):
            listIndex = nItemsAtStartOfCellFeats + cellIndex * 7
            category = int(cellFeatsList[listIndex])
            if category in (18, 19, 20):  # (white card, cannot est, background)
                continue
            if category not in colourDicts:
                colourDicts[category] = {'cellList':[]}
            cellDesc = [0.0] * 6
            #cellDesc[0] = float(cellFeatsList[listIndex+1]) * imgInvFeatsList[2]  # V (brightness)
            #cellDesc[1] = float(cellFeatsList[listIndex+2]) * imgInvFeatsList[2]
            #cellDesc[2] = float(cellFeatsList[listIndex+3]) * imgInvFeatsList[2]
            #cellDesc[3] = float(cellFeatsList[listIndex+4]) * imgInvFeatsList[2]
            #cellDesc[4] = float(cellFeatsList[listIndex+5]) * imgInvFeatsList[2]
            #cellDesc[5] = float(cellFeatsList[listIndex+6]) * imgInvFeatsList[2]
            cellDesc[0] = float(cellFeatsList[listIndex+1])
            cellDesc[1] = float(cellFeatsList[listIndex+2])
            cellDesc[2] = float(cellFeatsList[listIndex+3])
            cellDesc[3] = float(cellFeatsList[listIndex+4])
            cellDesc[4] = float(cellFeatsList[listIndex+5])
            cellDesc[5] = float(cellFeatsList[listIndex+6])
            colourDicts[category]['cellList'].append(cellDesc)

    # Create colour desc and add to dict for each colour.
    for colour, colourDict in colourDicts.items():
        nCells = len(colourDict['cellList'])
        groupDesc = [0.0] * 6
        for cellDesc in colourDict['cellList']:
            for i in range(6):
                groupDesc[i] += cellDesc[i]
        nCellsInv = 1.0 / nCells
        groupDesc = [x * nCellsInv for x in groupDesc]
        colourDict['desc'] = groupDesc
        print('Description of colour %d from %d cells...' % (colour, nCells))
        pprint.pprint(groupDesc)

        groupDescVar = [0.0] * 6
        for cellDesc in colourDict['cellList']:
            for i in range(6):
                groupDescVar[i] += math.fabs(cellDesc[i] - groupDesc[i])
        groupDescVar = [x * nCellsInv for x in groupDescVar]
        colourDict['desc'].extend(groupDescVar)
        print('Variance in features...')
        pprint.pprint(groupDescVar)
    return colourDicts


def writeRobotGroups(colourDicts):
    '''Write descriptions of robot colours for copying into the robot source code.'''
    outputFilename = '__coopLocParamsToCopyToCode__.txt'
    coopLocOutputFile = open(outputFilename, 'w')
    nColours = len(colourDicts.keys())
    maxColourIndex = max(colourDicts.keys())

    colourDescToId = [-1] * len(colourDicts.keys())
    colourIdToDesc = [-1] * (maxColourIndex + 1)
    i = 0
    for colour in colourDicts.keys():
        colourIdToDesc[colour] = i
        colourDescToId[i] = colour
        i += 1

    log('#define N_ROBOT_COLOURS %d //!< Number of trained robot colours.' % nColours, file=coopLocOutputFile)
    log('#define MAX_ROBOT_COLOUR_ID %d //!< Max robot colour index' % maxColourIndex, file=coopLocOutputFile)
    log('', file=coopLocOutputFile)
    
    str0 = ', '.join(repr(x) for x in colourDescToId)
    log('const int robotColourDescToId[%d] = {%s};' % (nColours, str0), file=coopLocOutputFile)
    
    str0 = ', '.join(repr(x) for x in colourIdToDesc)
    log('const int robotColourIdToDesc[%d] = {%s};' % (maxColourIndex + 1, str0), file=coopLocOutputFile)    
    log('', file=coopLocOutputFile)
    
    log('const float robotColourDescs[N_ROBOT_COLOURS][12] = {', file=coopLocOutputFile)
    for colour, colourDict in colourDicts.items():
        desc = colourDict['desc']
        log('{ // From %d cells' % len(colourDict['cellList']), file=coopLocOutputFile)
        log('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff,' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5]), file=coopLocOutputFile)
        log('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff' % (desc[6], desc[7], desc[8], desc[9], desc[10], desc[11]), file=coopLocOutputFile)
        log('},', file=coopLocOutputFile)
    log('};', file=coopLocOutputFile)
    log('', file=coopLocOutputFile)
    print('\nCoop loc blob descriptions written to %s' % outputFilename)

################################################################################################
#
# Image occupancy groups
#
################################################################################################


def __compareImageDescs(desc1, desc2):
    '''
    Features seem to be fairly even in terms of identifying occupancy, so an
    unweighted comparison should work ok.
    '''
    def __compareImageFeats(feat1, feat2):
        return math.fabs(feat1 - feat2) / (feat1 + feat2)
        
    featTuples = zip(desc1, desc2)
    diffs = (__compareImageFeats(x[0], x[1]) for x in featTuples)
    diff = sum(diffs)
    return diff

    
def __calculateImageGroupDescription(groupMembers, imageDescs):
    '''
    The image group description will just be a vector of the 12 float values.
    '''
    groupDesc = [0] * 12
    for index in groupMembers:
        imageDesc = imageDescs[index]
        for i in range(0, 12):
            groupDesc[i] += imageDesc[i]
    desc = [x/len(groupMembers) for x in groupDesc]
    return desc


@wrapFunction
def __mergeImageGroups(imageGroups):
    '''
    Merge image groups until nGroups is <= threshold. 
    
    Pretty arbitrary. Merge smaller groups with other groups that they are similar to.
    '''
    A = 10.0
    B = 1.0
    nGroups = len(imageGroups)
    if nGroups <= nImageGroupsMax:
        return imageGroups

    # Calculate attraction between all groups.
    groupDiffs = []
    for i in range(0, nGroups):
        group1 = imageGroups[i]
        for j in range(0, nGroups):
            group2 = imageGroups[j]
            if i == j or group1['nImages'] > group2['nImages'] or (j < i and group1['nImages'] == group2['nImages']):
                continue
            diff = __compareImageDescs(group1['desc'], group2['desc']) # Group description is same as that for images
            minSize = min(len(group1['members']), len(group2['members']))
            score = diff * A + minSize * B
            diffTuple = (score, i, j)
            groupDiffs.append(diffTuple)
    groupDiffs.sort(key = lambda x: x[0]) # Sort based on first element
    
    printMergeGroupDetails = True
    if printMergeGroupDetails:
        nMembers = sum((len(x['members']) for x in imageGroups))
        print('Initial nGroups %d nMembers %d' % (len(imageGroups), nMembers))
        ids = [x['id'] for x in imageGroups]
        ids.sort()
        print(ids)

    # Iterate through list of group comparison. Each is a tuple containing the similarity metric, and a pointer
    # to each of the groups. Either group may have been merged with another group, so the function __findEndGroup
    # is used to retrieve its current group.
    while nGroups > nImageGroupsMax and groupDiffs:
        (score, index1, index2) = groupDiffs.pop(0)
        sourceGroup = imageGroups[index1]
        destGroup = imageGroups[index2]
        sourceEndGroup = __findEndGroup(sourceGroup)
        destEndGroup = __findEndGroup(destGroup)
        assert sourceEndGroup == sourceGroup # If group had already been merged onto another, should have been removed.
        if sourceEndGroup != destEndGroup:
            if printMergeGroupDetails:
                print('Merge group %d(nMembers=%d) onto %d(%d)' % (index1, len(sourceGroup['members']), index2, len(destGroup['members'])))
            sourceGroup['dest'] = destGroup
            groupDiffs = [x for x in groupDiffs if x[1] != index1]
            nGroups -= 1

    # If a group has a 'dest' group, then it is to be merged with this group. Take its members and add to the
    # dest group.
    reducedGroups = []
    for group in imageGroups:
        if 'dest' in group.keys():
            __propogateMergedMembers(group)
        else:
            reducedGroups.append(group)
        
    if printMergeGroupDetails:
        nMembers = sum((len(x['members']) for x in reducedGroups))
        print('Reduced nGroups %d nMembers %d' % (len(reducedGroups), nMembers))
        ids = [x['id'] for x in reducedGroups]
        ids.sort()
        print(ids)

    fullList = []
    for group in reducedGroups:
        for member in group['members']:
            if member in fullList:
                print('DUPLICATE %d!!!' % member)
        fullList.extend(group['members'])

    return reducedGroups
    

@wrapFunction
def groupImages():
    '''
    Create dictionary of imageGroups. Each with a description and
    a set of images from which it was trained.
    '''
    nImages = len([x for x in os.listdir(obstDetDir) if x.startswith('cellFeatures')])
    assert nImages != 0, 'ERROR: nImages not set or no images found'
    
    # Get list of image descriptions
    imageDescs = []
    for index in range(0, nImages):
        filename = r'%s\imgFeatures%03d.txt' % (obstDetDir, index)
        imageFile = open(filename)
        imageFeats = imageFile.readlines()
        imageFeats = [float(y) for y in [x.strip() for x in imageFeats] if y != '']
        imageDescs.append(imageFeats)

    # Get diffs between all images
    imageDiffs = []
    for i in range(0, nImages):
        imageDesc = imageDescs[i]
        for j in range(i + 1, nImages):
            otherDesc = imageDescs[j]
            diff = __compareImageDescs(imageDesc, otherDesc)
            diffTuple = (diff, i, j)
            imageDiffs.append(diffTuple)
    imageDiffs.sort(key = lambda x: x[0]) # Compare diffTuples based on the first value
    
    # Group images. Continue until all images have been added to a group.
    imageGroups = []
    imagesAdded = []
    while imageDiffs and len(imagesAdded) < nImages:
        (diff, index1, index2) = imageDiffs.pop(0)
        if index1 in imagesAdded and index2 in imagesAdded:
            continue    
        isAdded = False
        for group in imageGroups:
            image1InGroup = index1 in group['members']
            image2InGroup = index2 in group['members']
            if image1InGroup and image2InGroup:
                isAdded = True
                break
            if image1InGroup:
                if not index2 in imagesAdded:
                    group['members'].append(index2)
                    imagesAdded.append(index2)
                isAdded = True
                break
            if image2InGroup:
                if not index1 in imagesAdded:
                    group['members'].append(index1)
                    imagesAdded.append(index1)
                isAdded = True
                break
        if not isAdded:
            newGroup = {}
            assert index1 not in imagesAdded and index2 not in imagesAdded
            newGroup['members'] = [index1, index2]
            newGroup['id'] = len(imageGroups)
            imageGroups.append(newGroup)
            imagesAdded.append(index1)
            imagesAdded.append(index2)

    # Create description for each group. Will be necessary to match an image against
    # a group when performing obstacle detection.
    for group in imageGroups:
        group['desc'] = __calculateImageGroupDescription(group['members'], imageDescs)
        group['nImages'] = len(group['members'])

    # Merge groups together based on similarity.
    imageGroups = __mergeImageGroups(imageGroups)

    # Re-calculate description of groups after merging
    for group in imageGroups:
        group['desc'] = __calculateImageGroupDescription(group['members'], imageDescs)
        group['nImages'] = len(group['members'])

    for group in imageGroups:
        print('Group %d(%d)' % (group['id'], len(group['members'])))
        print(group['members'])
        group.pop('id')

    return imageGroups, imageDescs
    
    
################################################################################################
#
# Cell occupancy groups
#
################################################################################################


@wrapFunction
def __calculateCellDescriptions(group):
    '''
    For each image in given image group, calculate ratios of cellFeatures to
    imgFeatures. These values are read from files.
    '''
    tempList = [0] * 6
    unoccupiedDescs = []
    occupiedDescs = []
    cellLists = (unoccupiedDescs, occupiedDescs)
    for index in group['members']:
        # Read cell and (inverse) image descriptions from files
        cellFeatsFilename = r'%s\cellFeatures%03d.txt' % (obstDetDir, index)
        imgInvFeatsFilename = r'%s\imgInvFeatures%03d.txt' % (obstDetDir, index)
    
        cellFeatsFile = open(cellFeatsFilename)
        imgInvFeatsFile = open(imgInvFeatsFilename)
        
        cellFeatsList = cellFeatsFile.readlines()
        imgInvFeatsList = imgInvFeatsFile.readlines()
        
        cellFeatsList = [y for y in (x.strip() for x in cellFeatsList) if y != '']
        imgInvFeatsList = [float(y) for y in (x.strip() for x in imgInvFeatsList) if y != '']
        
        nItemsAtStartOfCellFeats = 1
        nCellsThisImage = occupGridXDims * int(cellFeatsList[0]) # Read occupancy grid Y dimensions from start of list

        for cellIndex in range(0, nCellsThisImage):
            listIndex = nItemsAtStartOfCellFeats + cellIndex * 7

            isOccupied = int(cellFeatsList[listIndex])
            tempList[0] = float(cellFeatsList[listIndex+1])
            tempList[1] = float(cellFeatsList[listIndex+2])
            tempList[2] = float(cellFeatsList[listIndex+3])
            tempList[3] = float(cellFeatsList[listIndex+4])
            tempList[4] = float(cellFeatsList[listIndex+5])
            tempList[5] = float(cellFeatsList[listIndex+6])
            
            if isOccupied == 2: # Between occupied and unoccupied
                continue

            cellDesc = []
            for featIndex in range(0, 6):
                cellFeat = tempList[featIndex]
                ratioList = [0]*12
                for ratioIndex in range(0, 12):
                    # Reciprocals of these values calculated already to avoid divisions.
                    ratio = cellFeat * imgInvFeatsList[ratioIndex]
                    ratioList[ratioIndex] = ratio
                cellDesc.append(ratioList)
            cellLists[isOccupied].append(cellDesc)

    print('Group cells nUnoccup %d nOccup %d' % (len(unoccupiedDescs), len(occupiedDescs)))

    # Each cell of each image in this group has now been added to either the unoccupied or occupied list.
    return unoccupiedDescs, occupiedDescs
    
    
def __compareCellDescs(desc1, desc2, refFeats):
    def __compareCellFeats(feat1, feat2, refFeat):
        return math.fabs(feat1[refFeat] - feat2[refFeat])

    featTuples = zip(desc1, desc2, refFeats)
    diffs = (__compareCellFeats(x[0], x[1], x[2]) for x in featTuples)
    diff = sum(diffs)
    return diff
    

def __compareCellGroupDescs(desc1, desc2):
    diff = math.fabs(desc1[0] - desc2[0]) +  \
           math.fabs(desc1[1] - desc2[1]) +  \
           math.fabs(desc1[2] - desc2[2]) +  \
           math.fabs(desc1[3] - desc2[3]) +  \
           math.fabs(desc1[4] - desc2[4]) +  \
           math.fabs(desc1[5] - desc2[5]);
    return diff
    

def __calculateCellGroupDescription(groupMembers, cellDescs, refFeats):
    groupDesc = [0] * 6
    for cellIndex in groupMembers:
        cellDesc = cellDescs[cellIndex]
        for i in range(0, 6):
            groupDesc[i] += cellDesc[i][refFeats[i]]
    nGroupMembersInv = 1 / len(groupMembers)
    desc = [x * nGroupMembersInv for x in groupDesc]
    return desc

@wrapFunction
def __calculateReferenceFeatures(images, unoccupiedCells, occupiedCells):
    '''
    Calculate the image feature that best describes each cell feature for this group;
    the ratio with least variance. Cells can then be described against these reference features.
    
    As obstacle detection will obviously be carried out on un-categorised
    images, variance should be determined over both un- and occupied cells.
    '''    
    refFeats = []
    refScores = []
    for featIndex in range(0, 6):
        
        bestScore = -10000
        bestFeat = -1
        for refFeatIndex in range(0, 12):
        
            ratiosUnoccup = [x[featIndex][refFeatIndex] for x in unoccupiedCells]
            ratiosOccup = [x[featIndex][refFeatIndex] for x in occupiedCells]
            sumUnoccup = sum(ratiosUnoccup)
            sumOccup = sum(ratiosOccup)
            sumTotal = sumUnoccup + sumOccup
            lenTotal = len(ratiosUnoccup) + len(ratiosOccup)
            avgTotal = sumTotal / lenTotal
            if 0 == len(ratiosUnoccup) or 0 == len(ratiosOccup):
                diffNorm = 1.0
            else:
                avgUnoccup = sumUnoccup / len(ratiosUnoccup)
                avgOccup = sumOccup / len(ratiosOccup)
                diff = math.fabs(avgUnoccup - avgOccup)
                diffNorm = diff / avgTotal
            
            var = 0.0
            for ratio in ratiosUnoccup:
                var += math.fabs(avgTotal - ratio)
            for ratio in ratiosOccup:
                var += math.fabs(avgTotal - ratio)
            var /= lenTotal
            varNorm = var / avgTotal

            # Score is average diff between occup and unoccup cells over variance
            score = diffNorm / varNorm
            
            if score > bestScore:
                bestScore = score
                bestFeat = refFeatIndex

        refFeats.append(bestFeat)
        refScores.append(bestScore)

    print('Reference features')
    for i in range(0, 6):
        print('%2d Ref feat:%2d Score:%10f' % (i, refFeats[i], refScores[i]))
    
    return refFeats
    

@wrapFunction
def __mergeCellGroups(cellGroups):
    '''
    Merge cell groups until nGroups is <= threshold.
    '''
    A = 10.0
    B = 1.0
    nGroups = len(cellGroups)
    if nGroups <= nCellGroupsMax:
        return cellGroups

    startTimeTotal = time.clock()
    startTime = time.clock()
    
    # Calculate attraction between all groups. Smaller groups are merged onto
    # larger ones, so once a small group is merged, it can be removed from the list of potential
    # merge-pairs
    groupDiffs = []    
    for i in range(0, nGroups):
        group1 = cellGroups[i]
        #for j in range(0, nGroups):
        for j in range(i + 1, nGroups):
            group2 = cellGroups[j]
            #if i == j or group1['nCells'] > group2['nCells']:
            #    continue
            # Group description is same as that for images
            diff = __compareCellGroupDescs(group1['desc'], group2['desc'])
            #minSize = min(len(group1['members']), len(group2['members']))
            #score = diff * A + minSize * B
            score = diff
            diffTuple = (score, i, j)
            groupDiffs.append(diffTuple)

    groupDiffs.sort(key=lambda x: x[0], reverse=True)

    if False:
        print('group diffs %d' % len(groupDiffs))
        pprint.pprint(groupDiffs)
    
    printMergeGroupDetails = False
    if printMergeGroupDetails:
        nMembers = sum((len(x['members']) for x in cellGroups))
        print('Initial nGroups %d nMembers %d' % (len(cellGroups), nMembers))
        ids = [x['id'] for x in cellGroups]
        ids.sort()
        print(ids)
    
    getDiffsTime = time.clock() - startTime
    startTime = time.clock()
    
    while nGroups > nCellGroupsMax and groupDiffs:
        (score, index1, index2) = groupDiffs.pop()
        sourceGroup = cellGroups[index1]
        destGroup = cellGroups[index2]
        sourceEndGroup = __findEndGroup(sourceGroup)
        destEndGroup = __findEndGroup(destGroup)
        assert sourceEndGroup == sourceGroup # If group had already been merged onto another, should not be merged again
        if sourceEndGroup != destEndGroup:
            if printMergeGroupDetails:
                print('Merge group %d(nMembers=%d) onto %d(%d)' % (index1, len(sourceGroup['members']), index2, len(destGroup['members'])))
            sourceGroup['dest'] = destGroup
            groupDiffs = [x for x in groupDiffs if x[1] != index1]
            nGroups -= 1

    mergeTime = time.clock() - startTime
    startTime = time.clock()

    reducedGroups = []
    for group in cellGroups:
        if 'dest' in group.keys():
            __propogateMergedMembers(group)
        else:
            reducedGroups.append(group)

    propogateTime = time.clock() - startTime

    if False:
        for group in cellGroups:
            print('group %d' % len(group['members']))
            pprint.pprint(group['members'])


    if printMergeGroupDetails:
        nMembers = sum([len(x['members']) for x in reducedGroups])
        print('Reduced nGroups %d nMembers %d' % (len(reducedGroups), nMembers))
        ids = [x['id'] for x in reducedGroups]
        ids.sort()
        print(ids)

    fullList = []
    for group in reducedGroups:
        for member in group['members']:
            if member in fullList:
                print('DUPLICATE %d' % member)
        fullList.extend(group['members'])
    
    totalTime = time.clock() - startTimeTotal
    
    print('TIME TO CALC DIFFS %f' % getDiffsTime)
    print('TIME TO MERGE GROUPS %f' % mergeTime)
    print('TIME TO PROPOGATE MERGES %f' % propogateTime)
    print('TIME TOTAL %f' % totalTime)

    return reducedGroups

    
@wrapFunction
def __groupCells(cellDescs, refFeats):
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

    # If the number of cells in an image group is too large, split cells into list of
    # sub groups. Grouping cells is O(n!), therefore a list of smaller groups is much
    # quicker.
    allCellGroups = []
    cellDescSubLists = []
    nCellsTotal = len(cellDescs)
    if nCellsTotal < nCellsSplitIntoBlocks:
        subListTuple = (0, cellDescs)
        cellDescSubLists.append(subListTuple)
    else:
        i = 0
        while i < nCellsTotal:
            subList = cellDescs[i:i + nCellsBlock]
            subListTuple = (i, subList)
            cellDescSubLists.append(subListTuple)
            i += nCellsBlock
    
    getDiffsTime = 0
    groupTime = 0
    totalTime = 0
    
    startTimeTotal = time.clock()

    for startIndex, cellDescSubList in cellDescSubLists:

        startTime = time.clock()
        
        # For each cellDesc in this (sub)list, calculate simularity to each other celDesc. Store simularity
        # measures in order list.
        nCellsSubList = len(cellDescSubList)
        cellDiffs = []
        for i in range(0, nCellsSubList):
            cellDesc = cellDescSubList[i]
            otherDescs = ((cellDescSubList[x], x) for x in range(i + 1, nCellsSubList))
            subListDiffs = [(__compareCellDescs(cellDesc, x[0], refFeats), i, x[1]) for x in otherDescs]
            cellDiffs.extend(subListDiffs)
        cellDiffs.sort(key = lambda x: x[0]) # Compare diffTuples based on the first value
        
        if False:
            print('cell diffs')
            pprint.pprint(cellDiffs)
        
        elapsedTime = time.clock() - startTime        
        getDiffsTime += elapsedTime
        
        startTime = time.clock()
        
        # Group cell appearances together
        cellGroups = []
        cellsAdded = []
        while cellDiffs and len(cellsAdded) < nCellsSubList:
            (diff, index1, index2) = cellDiffs.pop(0)
            if index1 in cellsAdded and index2 in cellsAdded:
                continue
            isAdded = False
            for group in cellGroups:
                cell1InGroup = index1 in group['members']
                cell2InGroup = index2 in group['members']
                if cell1InGroup and cell2InGroup:
                    isAdded = True
                    break
                if cell1InGroup:
                    if not index2 in cellsAdded:
                        group['members'].append(index2)
                        cellsAdded.append(index2)
                    isAdded = True
                    break
                if cell2InGroup:
                    if not index1 in cellsAdded:
                        group['members'].append(index1)
                        cellsAdded.append(index1)
                    isAdded = True
                    break
            if not isAdded:
                newGroup = {}
                newGroup['members'] = [index1, index2]
                newGroup['id'] = len(cellGroups)
                assert index1 not in cellsAdded and index2 not in cellsAdded
                cellGroups.append(newGroup)    
                cellsAdded.append(index1)
                cellsAdded.append(index2)

        if False:
            for group in cellGroups:
                print('group %d' % len(group['members']))
                pprint.pprint(group['members'])

        for group in cellGroups:
            group['desc'] = __calculateCellGroupDescription(group['members'], cellDescs, refFeats)
            group['nCells'] = len(group['members'])
            group['members'] = [x + startIndex for x in group['members']]
        allCellGroups.extend(cellGroups)
        
        elapsedTime = time.clock() - startTime        
        groupTime += elapsedTime
        
        print('Cells from index %d grouped' % startIndex)

    elapsedTime = time.clock() - startTimeTotal
    print('TIME TOTAL %f' % elapsedTime)
        
    print('TIME TO CALC CELL DIFFS %f' % getDiffsTime)
    print('TIME TO DO INITIAL GROUPING OF CELLS %f' % groupTime)
    
    if False:
        print('n cell groups %d' % len(allCellGroups))
        for group in allCellGroups:
            pprint.pprint(group)

    mergedCellGroups = __mergeCellGroups(allCellGroups)
    
    for group in mergedCellGroups:
        group['desc'] = __calculateCellGroupDescription(group['members'], cellDescs, refFeats)
        group['nCells'] = len(group['members'])
        
    print('Cell groups')
    for group in mergedCellGroups:
        print('Group %3d (%d)' % (group['id'], len(group['members'])))
        group.pop('id')

    if False:
        print('n merged groups %d' % len(mergedCellGroups))
        for group in mergedCellGroups:
            pprint.pprint(group)

    return mergedCellGroups

    
@wrapFunction
def calculateOccupancyGroups(imageGroups, imageDescs):
    '''
    Have a set of image groups each with associated description. For each group, also
    have set of descriptions of unoccupied and occupied cells.
    
    As there may be multiple distinct appearances for occupied/unoccupied cells, cells
    are grouped together in groups labeled as occupied/unoccupied.
    '''
    haveCustomModule = 1
    if haveCustomModule:
        import imageUtils
        mergeFunction = imageUtils.groupCells
    else:
        mergeFunction = __groupCells

    index = 0
    for group in imageGroups:
        unoccupiedDesc, occupiedDesc = __calculateCellDescriptions(group)
        group['unoccupied'] = unoccupiedDesc
        group['occupied'] = occupiedDesc

        group['refFeats'] = __calculateReferenceFeatures(group['members'], group['unoccupied'], group['occupied'])
        
        if not group['unoccupied']:
            group['unoccupiedGroups'] = []
        else:
            group['unoccupiedGroups'] = mergeFunction(group['unoccupied'], group['refFeats'])

        if not group['occupied']:
            group['occupiedGroups'] = []
        else:
            group['occupiedGroups'] = mergeFunction(group['occupied'], group['refFeats'])

        index += 1

    # For each image group, the set of occupied and unoccupied cells are now arranged into groups.
    return 0


################################################################################################
#
# Testing
#
################################################################################################


@wrapFunction
def writeImageGroupsCode(imageGroups):
    trainedModelOutputFile = open('__obstDetParamsToCopyToCode__.txt', 'w')

    log('#define N_IMAGE_GROUPS %d' % len(imageGroups), file=trainedModelOutputFile)
    log('const float imgGroupDescs[%d][12] = {' % len(imageGroups), file=trainedModelOutputFile)
    for imageGroup in imageGroups:
        # Remember to remove last ',' when copying into code!
        desc = imageGroup['desc'] 
        log('{ // From %d images' % imageGroup['nImages'], file=trainedModelOutputFile)
        log('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff,' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5]), file=trainedModelOutputFile)
        log('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff' % (desc[6], desc[7], desc[8], desc[9], desc[10], desc[11]), file=trainedModelOutputFile)
        log('},', file=trainedModelOutputFile)
    log('};', file=trainedModelOutputFile)
    log('', file=trainedModelOutputFile)
    
    log('const int imgGroupRefFeats[%d][6] = {' % len(imageGroups), file=trainedModelOutputFile)
    for imageGroup in imageGroups:
        feats = imageGroup['refFeats']
        log('{%2d, %2d, %2d, %2d, %2d, %2d},' % (feats[0], feats[1], feats[2], feats[3], feats[4], feats[5]), file=trainedModelOutputFile)
    log('};', file=trainedModelOutputFile)
    log('', file=trainedModelOutputFile)

    index = 0
    for imageGroup in imageGroups:
        log('const int nUnoccupiedCellGroupsForImgGroup%d = %d;' % (index, len(imageGroup['unoccupiedGroups'])), file=trainedModelOutputFile)
        if len(imageGroup['unoccupiedGroups']):
            log('const float unoccupiedCellGroupsForImgGroup%d[%d][6] = {' % (index, len(imageGroup['unoccupiedGroups'])), file=trainedModelOutputFile)
            for cellGroup in imageGroup['unoccupiedGroups']:
                desc = cellGroup['desc']
                log('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // From %d cells' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], cellGroup['nCells']), file=trainedModelOutputFile)
            log('};', file=trainedModelOutputFile)
        log('const int nOccupiedCellGroupsForImgGroup%d = %d;' % (index, len(imageGroup['occupiedGroups'])), file=trainedModelOutputFile)
        if len(imageGroup['occupiedGroups']):
            log('const float occupiedCellGroupsForImgGroup%d[%d][6] = {' % (index, len(imageGroup['occupiedGroups'])), file=trainedModelOutputFile)
            for cellGroup in imageGroup['occupiedGroups']:
                desc = cellGroup['desc']
                log('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // From %d cells' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], cellGroup['nCells']), file=trainedModelOutputFile)
            log('};', file=trainedModelOutputFile)
        log('', file=trainedModelOutputFile)
        index += 1
    
    # Also write out function to return the appropriate list given a group index. This has to be updated
    # by hand otherwise.
    log('\n\n\n\n\n\n\n', file=trainedModelOutputFile)
    log('\t\t\t// Get set of cellGroups for this imageGroup', file=trainedModelOutputFile)
    log('\t\t\tswitch (imgGroupIndex)', file=trainedModelOutputFile)
    log('\t\t\t{', file=trainedModelOutputFile)
    
    for i in range(len(imageGroups)):
        log('\t\t\tcase %d:' % i, file=trainedModelOutputFile)

        imageGroup = imageGroups[i]
        log('\t\t\t\tnUnoccupCellGroups = nUnoccupiedCellGroupsForImgGroup%d;' % i, file=trainedModelOutputFile)
        log('\t\t\t\tnOccupCellGroups = nOccupiedCellGroupsForImgGroup%d;' % i, file=trainedModelOutputFile)
        
        cellGroup = '0';
        if len(imageGroup['unoccupiedGroups']):
            cellGroup = 'unoccupiedCellGroupsForImgGroup%d' % i
        log('\t\t\t\tunoccupGroupDescs = %s;' % cellGroup, file=trainedModelOutputFile)
            
        cellGroup = '0';
        if len(imageGroup['occupiedGroups']):
            cellGroup = 'occupiedCellGroupsForImgGroup%d' % i
        log('\t\t\t\toccupGroupDescs = %s;' % cellGroup, file=trainedModelOutputFile)
        
        log('\t\t\t\tbreak;', file=trainedModelOutputFile)

    log('\t\t\tdefault:', file=trainedModelOutputFile)
    log('\t\t\t\tnUnoccupCellGroups = 0;', file=trainedModelOutputFile)
    log('\t\t\t\tnOccupCellGroups = 0;', file=trainedModelOutputFile)
    log('\t\t\t\tunoccupGroupDescs = 0;', file=trainedModelOutputFile)
    log('\t\t\t\toccupGroupDescs = 0;', file=trainedModelOutputFile)
    log('\t\t\t\tbreak;', file=trainedModelOutputFile)
    log('\t\t\t}', file=trainedModelOutputFile)
    
    return 0


@wrapFunction
def writeImageGroups(imageGroups):
    for group in imageGroups:
        group.pop('occupied')
        group.pop('unoccupied')
        if group['unoccupiedGroups']:
            group['unoccupiedGroups'].sort(key = lambda x: x['desc'][0]) # Sort based on first element in 'desc'
            if 'members' in group['unoccupiedGroups'][0]:
                for cellGroup in group['unoccupiedGroups']:
                    # Remove elements from dict that we do not want to pickle
                    cellGroup.pop('members')
        if group['occupiedGroups']:
            group['occupiedGroups'].sort(key = lambda x: x['desc'][0]) # Sort based on first element in 'desc'
            if 'members' in group['occupiedGroups'][0]:
                for cellGroup in group['occupiedGroups']:
                    cellGroup.pop('members')

    outputFile = open(r'%s\__imageGroups__.p' % obstDetDir, 'wb')
    pickle.dump(imageGroups, outputFile, protocol=2)
    
    outputFile = open(r'%s\__imageGroups__.txt' % obstDetDir, 'w')
    pprint.pprint(imageGroups, outputFile)    
    
    writeImageGroupsCode(imageGroups)
    analyseGroups(imageGroups)
    
    
@wrapFunction
def loadImageGroups():
    inputFile = open(r'%s\__imageGroups__.p' % obstDetDir, 'rb')
    imageGroups = pickle.load(inputFile)
    return imageGroups
    

@wrapFunction
def analyseGroups(imageGroups):
    outputFile = open(r'%s\__imageGroupsAnalysis__.txt' % obstDetDir, 'w')
    
    imageGroupsDone = []
    allGroupsDiffs = []
    log('~'*32, file=outputFile)
    log('Image group diffs', file=outputFile)
    log('~'*32, file=outputFile)

    i = 0
    for imageGroup in imageGroups:
        groupDiffs = []
        imageGroupsDone.append(imageGroup)
        groupDesc = imageGroup['desc']
        for otherGroup in imageGroups:
            if otherGroup not in imageGroupsDone:
                diff = __compareImageDescs(otherGroup['desc'], groupDesc)
                groupDiffs.append(diff)
        if groupDiffs:
            allGroupsDiffs.append(groupDiffs)
            log(str(i), file=outputFile)
            i+=1
            pprint.pprint(groupDiffs, outputFile)
    log('', file=outputFile)
    
    i = 0
    for imageGroup in imageGroups:
        internalGroupDiffs = []
        log('~'*32, file=outputFile)
        log('Image group %d details' % i, file=outputFile)
        i+=1
        log('~'*32, file=outputFile)
        log('', file=outputFile)
        log('Group members (%d)' % imageGroup['nImages'], file=outputFile)
        pprint.pprint(imageGroup['members'], outputFile)
        log('', file=outputFile)
        
        log('Unoccupied cell groups (%d), diffs between' % len(imageGroup['unoccupiedGroups']), file=outputFile)
        unoccupGroupsDone = []
        allGroupsDiffs = []
        for cellGroup in imageGroup['unoccupiedGroups']:
            groupDiffs = []
            unoccupGroupsDone.append(cellGroup)
            groupDesc = cellGroup['desc']
            for otherGroup in imageGroup['unoccupiedGroups']:
                if otherGroup not in unoccupGroupsDone:
                    diff = __compareCellGroupDescs(groupDesc, otherGroup['desc'])
                    groupDiffs.append(diff)
            if groupDiffs:
                allGroupsDiffs.append(groupDiffs)
        if allGroupsDiffs:
            pprint.pprint(allGroupsDiffs, outputFile)
        log('', file=outputFile)
        
        log('Occupied cell groups (%d), diffs between' % len(imageGroup['occupiedGroups']), file=outputFile)
        occupGroupsDone = []
        allGroupsDiffs = []
        for cellGroup in imageGroup['occupiedGroups']:
            groupDiffs = []
            occupGroupsDone.append(cellGroup)
            groupDesc = cellGroup['desc']
            for otherGroup in imageGroup['occupiedGroups']:
                if otherGroup not in occupGroupsDone:
                    diff = __compareCellGroupDescs(groupDesc, otherGroup['desc'])
                    groupDiffs.append(diff)
            if groupDiffs:
                allGroupsDiffs.append(groupDiffs)
        if allGroupsDiffs:
            pprint.pprint(allGroupsDiffs, outputFile)
        log('', file=outputFile)
    return 0
    

@wrapFunction
def testObstacleDetection(imageGroups):
    imageDiffCoeff = 2;
    nEsts = 0
    nPos = 0
    nNeg = 0
    nCorrect = 0
    nFalsePos = 0
    nFalseNeg = 0
    bestUnoccupImgGroup = 0
    bestUnoccupCellGroup = 0
    bestOccupImgGroup = 0
    bestOccupCellGroup = 0
    falseDict = {}
    
    nImages = len([x for x in os.listdir(obstDetDir) if x.startswith('cellFeatures')])
    assert nImages != 0, 'ERROR: nImages not set or no images found'    
    
    cellDesc = [0] * 6
    for index in range(0, nImages):
    #for index in range(0, 1):
    
        # Get closeness of each image group
        filename = r'%s\imgFeatures%03d.txt' % (obstDetDir, index)
        imageFile = open(filename)
        imageFeats = imageFile.readlines()
        imageFeats = [float(y) for y in (x.strip() for x in imageFeats) if y != '']

        for imageGroup in imageGroups:
            diff = __compareImageDescs(imageFeats, imageGroup['desc'])
            imageGroup['diff'] = diff

        # Compare each cell in image to cellGroups in imageGroups
        filename = r'%s\cellFeatures%03d.txt' % (obstDetDir, index)
        cellFile = open(filename)
        cellFeatsList = cellFile.readlines()
        cellFeatsList = [y for y in [x.strip() for x in cellFeatsList] if y != '']

        filename = r'%s\imgInvFeatures%03d.txt' % (obstDetDir, index)
        imgInvFile = open(filename)
        imgInvFeatsList = imgInvFile.readlines()
        imgInvFeatsList = [float(y) for y in [x.strip() for x in imgInvFeatsList] if y != '']
        
        nItemsAtStartOfCellFeats = 1
        nCellsThisImage = occupGridXDims * int(cellFeatsList[0])

        for cellIndex in range(0, nCellsThisImage):
            listIndex = nItemsAtStartOfCellFeats + cellIndex * 7

            isOccupied = int(cellFeatsList[listIndex])
            cellDesc[0] = float(cellFeatsList[listIndex+1])
            cellDesc[1] = float(cellFeatsList[listIndex+2])
            cellDesc[2] = float(cellFeatsList[listIndex+3])
            cellDesc[3] = float(cellFeatsList[listIndex+4])
            cellDesc[4] = float(cellFeatsList[listIndex+5])
            cellDesc[5] = float(cellFeatsList[listIndex+6])
            
            comparrisonResultsOcc = []
            comparrisonResultsUnocc = []
            for imageGroup in imageGroups:
                imageDiff = imageGroup['diff']
                refFeats = imageGroup['refFeats']
                cellDescRelToImg = [cellDesc[i] * imgInvFeatsList[refFeats[i]] for i in range(0, 6)]
                
                for unoccupiedGroup in imageGroup['unoccupiedGroups']:
                    cellDiff = __compareCellGroupDescs(unoccupiedGroup['desc'], cellDescRelToImg)
                    diff = (cellDiff + imageDiff * imageDiffCoeff)
                    comparrisonResultsUnocc.append((imageDiff, cellDiff, diff))
                    
                for occupiedGroup in imageGroup['occupiedGroups']:
                    cellDiff = __compareCellGroupDescs(occupiedGroup['desc'], cellDescRelToImg)
                    diff = (cellDiff + imageDiff * imageDiffCoeff)
                    comparrisonResultsOcc.append((imageDiff, cellDiff, diff))
            
            comparrisonResultsUnocc.sort(key = lambda x: x[2])
            comparrisonResultsOcc.sort(key = lambda x: x[2])
            
            isOccupiedEst = comparrisonResultsOcc[0][2] < comparrisonResultsUnocc[0][2]
            isCorrect = (isOccupied == isOccupiedEst)
            
            
            falsePos = isOccupiedEst and (not isOccupied)
            falseNeg = (not isOccupiedEst) and isOccupied
            nEsts += 1
            if isOccupied:
                nPos += 1
            else:
                nNeg += 1
            if isCorrect:
                nCorrect += 1
            else:
                if index not in falseDict.keys():
                    falseDict[index] = 1
                else:
                    falseDict[index] += 1
            if falsePos:
                nFalsePos += 1
            if falseNeg:
                nFalseNeg += 1

            if not isCorrect:
                #print('index %s isOccupied %d occup %f unoccup %f' % (index, isOccupied, comparrisonResultsOcc[0][2], comparrisonResultsUnocc[0][2]))
                pass
            #print('%3d %d(%d) %12f %12f bestUnocImg %3d bestUnocCell %3d bestOcImg %3d bestOccCell %3d' % (cellIndex, isOccupiedEst, isCorrect, comparrisonResultsUnocc[0][2], comparrisonResultsOcc[0][2], comparrisonResultsUnocc[0][3], comparrisonResultsUnocc[0][4], comparrisonResultsOcc[0][3], comparrisonResultsOcc[0][4]))

    print('nEsts %d' % nEsts)
    print('nPos %d' % nPos)
    print('nNeg %d' % nNeg)
    print('nCorrect %d' % nCorrect)
    print('nFalsePos %d' % nFalsePos)
    print('nFalseNeg %d' % nFalseNeg)
    print('False cells in images:')
    pprint.pprint(falseDict)

    
if __name__ == '__main__':
    startTimeMain = time.clock()
    trainCoopLoc = 1
    trainObstDet = 0
    testCoopLoc = 0
    testObstDet = 0
    
    if trainCoopLoc:
        robotGroups = calculateRobotGroups()
        writeRobotGroups(robotGroups)
    
    elif trainObstDet:
        imageGroups, imageDescs = groupImages()
        calculateOccupancyGroups(imageGroups, imageDescs)
        writeImageGroups(imageGroups)

    elif testCoopLoc:
        pass
        
    elif testObstDet:
        imageGroups = loadImageGroups()
        testObstacleDetection(imageGroups)

    mainTime = time.clock() - startTimeMain
    print('OVERALL TIME %f' % mainTime)