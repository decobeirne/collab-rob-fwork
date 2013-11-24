import sys
import os.path
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir))
import math 
import pprint
from pdb import set_trace
import robotVisionUtils
import utils

LOG = utils.log
WRAP_FUNCTION = utils.wrapFunction

def compareImageDescs(desc1, desc2):
    '''
    Features seem to be fairly even in terms of identifying occupancy, so an
    unweighted comparison should work ok.
    '''
    def compareImageFeats(feat1, feat2):
        return math.fabs(feat1 - feat2) / (feat1 + feat2)
        
    featTuples = zip(desc1, desc2)
    diffs = (compareImageFeats(x[0], x[1]) for x in featTuples)
    diff = sum(diffs)
    return diff

def compareImageDescsNEW(desc1, desc2, featStdDevs):
    '''
    Features seem to be fairly even in terms of identifying occupancy, so an
    unweighted comparison should work ok.
    '''
    def compareImageFeats(feat1, feat2, featStdDev):
        diff = math.fabs(feat1 - feat2)
        return diff / featStdDev
        
    diffs = (compareImageFeats(x[0], x[1], x[2]) for x in zip(desc1, desc2, featStdDevs))
    diff = sum(diffs)
    return diff

def calculateImageGroupDescription(groupMembers, imageDescs):
    '''
    The image group description will just be a vector of the 12 float values.
    '''
    groupDesc = [0] * 12
    for index in groupMembers:
        imageDesc = imageDescs[index]
        for i in range(0, 12):
            groupDesc[i] += imageDesc[i]

    nMembersInv = float(1) / len(groupMembers)
    desc = [x * nMembersInv for x in groupDesc]

    return desc

def calculateImageGroupError(groupMembers, groupDesc, imageDescs, featStdDevs):
    groupError = 0
    for groupMember in groupMembers:
        diff = compareImageDescsNEW(imageDescs[groupMember], groupDesc, featStdDevs)
        groupError += diff

    #groupError *= (2 / len(groupMembers))
    return groupError

def loadImageDescs(dataDir):
    # Get list of image descriptions
    nImages = robotVisionUtils.getNObstRecTrainingImages(dataDir)

    imageDescs = []
    for index in range(0, nImages):
        filename = r'%s\imgFeatures%03d.txt' % (dataDir, index)
        imageFile = open(filename)
        imageFeats = imageFile.readlines()
        imageFeats = [float(y) for y in [x.strip() for x in imageFeats] if y != '']
        imageDescs.append(imageFeats)
    return imageDescs

@WRAP_FUNCTION
def groupImagesNEW3(dataDir, outputFile):
    '''
    Create dictionary of imageGroups. Each with a description and
    a set of images from which it was trained.
    '''
    nImages = robotVisionUtils.getNObstRecTrainingImages(dataDir)
    
    imageDescs = loadImageDescs(dataDir)
    
    # Get mean and stdDev for each feature over the set of image descriptions
    # Assumption that features with large stdDev will differentiate between images/groups
    featAvgs = []
    featStdDevs = []
    for feat in range(0, robotVisionUtils.N_IMG_FEATS):
        avg = 0
        for imageDesc in imageDescs:
            avg += imageDesc[feat]
        avg /= len(imageDescs)
        featAvgs.append(avg)
        stdDev = 0
        for imageDesc in imageDescs:
            stdDev += abs(imageDesc[feat] - avg)
        stdDev /= len(imageDescs)
        featStdDevs.append(stdDev)
    
    featNormStdDevs = []
    for avg, stdDev in zip(featAvgs, featStdDevs):
        normStdDev = stdDev / avg
        featNormStdDevs.append(normStdDev)
    
    LOG('!Avg, stdDev, normStdDev for each feature across all image descs...', file=outputFile)
    LOG('Image group feature avgs:', file=outputFile)
    pprint.pprint(featAvgs, stream=outputFile)
    LOG('Image group feature std devs:', file=outputFile)
    pprint.pprint(featStdDevs, stream=outputFile)
    LOG('Image group feature normalised std devs:', file=outputFile)
    pprint.pprint(featNormStdDevs, stream=outputFile)

    # Get diffs between all images
    imageDiffs = []
    for i in range(0, nImages):
        imageDesc = imageDescs[i]
        for j in range(i + 1, nImages):
            otherDesc = imageDescs[j]
            diff = compareImageDescsNEW(imageDesc, otherDesc, featStdDevs)
            diffTuple = (diff, 0, (i, 'image'), (j, 'image'))
            imageDiffs.append(diffTuple)
    
    # Compare diffTuples based on the first value
    imageDiffs.sort(key = lambda x: x[0] + x[1], reverse=True)
    
    # Group images. Continue until all images have been added to a group.
    imageGroups = {}
    nextImageGroupId = 0
    imageDescsNotYetAdded = [x for x in range(len(imageDescs))]
    #while imageDescsNotYetAdded and (len(imageGroups) + len(imageDescsNotYetAdded)) > robotVisionUtils.N_IMAGE_GROUPS_MAX:
    while (len(imageGroups) + len(imageDescsNotYetAdded)) > robotVisionUtils.N_IMAGE_GROUPS_MAX:
        imageDiff = imageDiffs.pop()
        (diff, groupError, item1, item2) = imageDiff
        (id1, type1) = item1
        (id2, type2) = item2
        LOG('Creating group %3d from diff: %s' % (nextImageGroupId, repr(imageDiff)), file=outputFile)
        newGroup = {}
        if type1 == 'group':
            newGroup['members'] = imageGroups[id1]['members']
            imageGroups.pop(id1)
        else:
            newGroup['members'] = [id1]
            imageDescsNotYetAdded.remove(id1)
        if type2 == 'group':
            newGroup['members'].extend(imageGroups[id2]['members'])
            imageGroups.pop(id2)
        else:
            newGroup['members'].append(id2)
            imageDescsNotYetAdded.remove(id2)
        newGroup['id'] = nextImageGroupId
        imageGroups[nextImageGroupId] = newGroup
        nextImageGroupId += 1
        newGroup['desc'] = calculateImageGroupDescription(newGroup['members'], imageDescs)
        newGroup['error'] = calculateImageGroupError(newGroup['members'], newGroup['desc'], imageDescs, featStdDevs)
        thisItem = (newGroup['id'], 'group')
        for i in range(len(imageDiffs)):
            otherDiff = imageDiffs[i]
            if (item1 in otherDiff) or (item2 in otherDiff):
                if otherDiff[2] == item1 or otherDiff[2] == item2:
                    otherItem = otherDiff[3]
                else:
                    otherItem = otherDiff[2]
                if otherItem[1] == 'image':
                    newDiff = compareImageDescsNEW(newGroup['desc'], imageDescs[otherItem[0]], featStdDevs)
                    newError = newGroup['error']
                else:
                    newDiff = compareImageDescsNEW(newGroup['desc'], imageGroups[otherItem[0]]['desc'], featStdDevs)
                    newError = newGroup['error'] + imageGroups[otherItem[0]]['error']
                newImageDiff = (newDiff, newError, thisItem, otherItem)
                #LOG('Diff %d %s -> %s' % (i, repr(otherDiff), repr(newImageDiff)), file=outputFile)
                imageDiffs[i] = newImageDiff

        # Uniqueify, then reverse sort based on item 0 in tuple
        oldLen = len(imageDiffs)
        imageDiffs = list(set(imageDiffs))
        imageDiffs.sort(key = lambda x: x[0] + x[1], reverse=True)
        LOG('Updated new group in imageDiffs, len %d->%d' % (oldLen, len(imageDiffs)), file=outputFile)
    
    # Put any remaining images into groups on their own - need to grab more images to fill out these groups
    LOG('Images not joined to any groups:', file=outputFile)
    pprint.pprint(imageDescsNotYetAdded, stream=outputFile)
    
    for imageId in imageDescsNotYetAdded:
        newGroup = {'members': [imageId], 'desc': imageDescs[imageId], 'error':0}
        newGroup['id'] = nextImageGroupId
        imageGroups[nextImageGroupId] = newGroup
        nextImageGroupId += 1

    LOG('PUT MERGE GROUP STEP IN HERE AGAIN', file=outputFile)

    LOG('%d image groups' % len(imageGroups), file=outputFile)
    for groupKey in imageGroups:
        group = imageGroups[groupKey]
        LOG('Group %d (%d)' % (group['id'], len(group['members'])), file=outputFile)
        LOG(repr(group['members']), file=outputFile)
        group.pop('id')

    return imageGroups, imageDescs, featStdDevs


@WRAP_FUNCTION
def checkImageGroups(imageGroups, imageDescs, featStdDevs, outputFile):
    '''
    Sanity check to verify that the groups make sense
    '''

    LOG('! Compare image descs to groups...', file=outputFile)
    nWrong = 0
    stats = []
    for i in range(0, len(imageDescs)):
        imageDesc = imageDescs[i]
        LOG('Image %d' % i, file=outputFile)
        diffsStr = []
        minDiff = 10000
        minOtherDiff = 10000
        groupDiff = 10000
        minIndex = -1
        groupIndex = -1
        for key, imageGroup in imageGroups.items():
            diff = compareImageDescsNEW(imageDesc, imageGroup['desc'], featStdDevs)
            inGroup = 0
            if i in imageGroup['members']:
                groupIndex = key
                groupDiff = diff
                inGroup = 1
            else:
                if diff < minOtherDiff:
                    minOtherDiff = diff
            if diff < minDiff:
                minDiff = diff
                minIndex = key
            minDiff = min(diff, minDiff)
            diffsStr.append(' %2d%s' % (int(diff), '~' if inGroup else ','))
        diffsStr.append(' min=%2d(%2d) group=%2d(%2d)' % (minIndex, minDiff, groupIndex, groupDiff))
        if minIndex != groupIndex:
            diffsStr.append(' WRONG(%d)' % (groupDiff - minDiff))
            nWrong += 1
        LOG(''.join(diffsStr), file=outputFile)
        stats.append((groupDiff, minOtherDiff))
    LOG('COMP_IMAGE_TO_GROUPS nWrong=%d nImages=%d %%=%f' % (nWrong, len(imageDescs), nWrong/len(imageDescs)), file=outputFile)
    
    LOG('GROUP_QUALITY')
    avgOwnDiff = 0
    avgOtherDiff = 0
    avgRatio = 0
    for tup in stats:
        LOG('%f,%f,%f,' % (tup[0], tup[1], tup[0] / tup[1]), file=outputFile)
        avgOwnDiff += tup[0]
        avgOtherDiff += tup[1]
        avgRatio += tup[0] / tup[1]
    avgOwnDiff /= len(stats)
    avgOtherDiff /= len(stats)
    avgRatio /= len(stats)
    LOG('', file=outputFile)
    LOG('%f,%f,%f,' % (avgOwnDiff, avgOtherDiff, avgRatio), file=outputFile)

    LOG('IMAGE_GROUP_ERRORS', file=outputFile)
    groupErrorList = [imageGroups[x]['error'] for x in imageGroups]
    for x in imageGroups:
        LOG('%f,%f,%f,' % (imageGroups[x]['error'], len(imageGroups[x]['members']), imageGroups[x]['error'] / len(imageGroups[x]['members'])), file=outputFile)
        
    LOG(repr(groupErrorList), file=outputFile)
    LOG('TOTAL_IMAGE_GROUP_ERROR sum=%f avg=%f' % (sum(groupErrorList), sum(groupErrorList) / len(groupErrorList)), file=outputFile)

    LOG('Compare image groups to eachother:', file=outputFile)
    keys = list(imageGroups.keys())
    nKeys = len(keys)
    groupsStr = [' %2d,' % x for x in keys[1:]]
    LOG('   ' + ''.join(groupsStr), file=outputFile)
    LOG('', file=outputFile)
    smallDiffs = []

    for i in range(0, nKeys):
        diffsStr = []
        diffsStr.append('%2d:' % keys[i] + '_' * (4 * i))
        for j in range(i + 1, nKeys):
            diff = compareImageDescsNEW(imageGroups[keys[i]]['desc'], imageGroups[keys[j]]['desc'], featStdDevs)
            diffsStr.append(' %2d,' % diff)
            if diff < 3:
                smallDiffs.append((keys[i], keys[j], diff))
        LOG(''.join(diffsStr), file=outputFile)
    LOG('%d Image groups are too similar:' % len(smallDiffs), file=outputFile)
    for i, j, diff in smallDiffs:
        LOG('Diff between %d and %d = %d' % (i, j, diff), file=outputFile)
        pprint.pprint(imageGroups[i]['desc'], stream=outputFile)
        pprint.pprint(imageGroups[j]['desc'], stream=outputFile)

@WRAP_FUNCTION
def writeImageGroupsCode(imageGroups):
    trainedModelOutputFile = open('__obstDetParamsToCopyToCode__.txt', 'w')

    LOG('#define N_IMAGE_GROUPS %d' % len(imageGroups), file=trainedModelOutputFile)
    LOG('const float imgGroupDescs[%d][12] = {' % len(imageGroups), file=trainedModelOutputFile)
    for imageGroup in imageGroups:
        # Remember to remove last ',' when copying into code!
        desc = imageGroup['desc'] 
        LOG('{ // From %d images' % imageGroup['nImages'], file=trainedModelOutputFile)
        LOG('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff,' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5]), file=trainedModelOutputFile)
        LOG('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff' % (desc[6], desc[7], desc[8], desc[9], desc[10], desc[11]), file=trainedModelOutputFile)
        LOG('},', file=trainedModelOutputFile)
    LOG('};', file=trainedModelOutputFile)
    LOG('', file=trainedModelOutputFile)

    LOG('const int imgGroupRefFeats[%d][6] = {' % len(imageGroups), file=trainedModelOutputFile)
    for imageGroup in imageGroups:
        feats = imageGroup['refFeats']
        LOG('{%2d, %2d, %2d, %2d, %2d, %2d},' % (feats[0], feats[1], feats[2], feats[3], feats[4], feats[5]), file=trainedModelOutputFile)
    LOG('};', file=trainedModelOutputFile)
    LOG('', file=trainedModelOutputFile)

    index = 0
    for imageGroup in imageGroups:
        LOG('const int nUnoccupiedCellGroupsForImgGroup%d = %d;' % (index, len(imageGroup['unoccupiedGroups'])), file=trainedModelOutputFile)
        if len(imageGroup['unoccupiedGroups']):
            LOG('const float unoccupiedCellGroupsForImgGroup%d[%d][6] = {' % (index, len(imageGroup['unoccupiedGroups'])), file=trainedModelOutputFile)
            for cellGroup in imageGroup['unoccupiedGroups']:
                desc = cellGroup['desc']
                LOG('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // From %d cells' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], cellGroup['nCells']), file=trainedModelOutputFile)
            LOG('};', file=trainedModelOutputFile)
        LOG('const int nOccupiedCellGroupsForImgGroup%d = %d;' % (index, len(imageGroup['occupiedGroups'])), file=trainedModelOutputFile)
        if len(imageGroup['occupiedGroups']):
            LOG('const float occupiedCellGroupsForImgGroup%d[%d][6] = {' % (index, len(imageGroup['occupiedGroups'])), file=trainedModelOutputFile)
            for cellGroup in imageGroup['occupiedGroups']:
                desc = cellGroup['desc']
                LOG('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // From %d cells' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], cellGroup['nCells']), file=trainedModelOutputFile)
            LOG('};', file=trainedModelOutputFile)
        LOG('', file=trainedModelOutputFile)
        index += 1
    
    # Also write out function to return the appropriate list given a group index. This has to be updated
    # by hand otherwise.
    LOG('\n\n\n\n\n\n\n', file=trainedModelOutputFile)
    LOG('\t\t\t// Get set of cellGroups for this imageGroup', file=trainedModelOutputFile)
    LOG('\t\t\tswitch (imgGroupIndex)', file=trainedModelOutputFile)
    LOG('\t\t\t{', file=trainedModelOutputFile)
    
    for i in range(len(imageGroups)):
        LOG('\t\t\tcase %d:' % i, file=trainedModelOutputFile)

        imageGroup = imageGroups[i]
        LOG('\t\t\t\tnUnoccupCellGroups = nUnoccupiedCellGroupsForImgGroup%d;' % i, file=trainedModelOutputFile)
        LOG('\t\t\t\tnOccupCellGroups = nOccupiedCellGroupsForImgGroup%d;' % i, file=trainedModelOutputFile)
        
        cellGroup = '0';
        if len(imageGroup['unoccupiedGroups']):
            cellGroup = 'unoccupiedCellGroupsForImgGroup%d' % i
        LOG('\t\t\t\tunoccupGroupDescs = %s;' % cellGroup, file=trainedModelOutputFile)
            
        cellGroup = '0';
        if len(imageGroup['occupiedGroups']):
            cellGroup = 'occupiedCellGroupsForImgGroup%d' % i
        LOG('\t\t\t\toccupGroupDescs = %s;' % cellGroup, file=trainedModelOutputFile)
        
        LOG('\t\t\t\tbreak;', file=trainedModelOutputFile)

    LOG('\t\t\tdefault:', file=trainedModelOutputFile)
    LOG('\t\t\t\tnUnoccupCellGroups = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\tnOccupCellGroups = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\tunoccupGroupDescs = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\toccupGroupDescs = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\tbreak;', file=trainedModelOutputFile)
    LOG('\t\t\t}', file=trainedModelOutputFile)
    
    return 0

@WRAP_FUNCTION
def writeImageGroupsCodeNEW(imageGroups, imageStdDevs):
    trainedModelOutputFile = open('__obstRecParamsToCopyToCode__.txt', 'w')
    
    LOG('#define CELL_DIFF_COEFF %ff' % robotVisionUtils.CELL_DIFF_COEFF, file=trainedModelOutputFile)
    LOG('#define IMAGE_DIFF_COEFF %ff' % robotVisionUtils.IMAGE_DIFF_COEFF, file=trainedModelOutputFile)
    LOG('#define N_IMAGE_GROUPS %d' % len(imageGroups), file=trainedModelOutputFile)
    LOG('const float imgStdDevs[12] = {', file=trainedModelOutputFile)
    LOG('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff,' % (imageStdDevs[0], imageStdDevs[1], imageStdDevs[2], imageStdDevs[3], imageStdDevs[4], imageStdDevs[5]), file=trainedModelOutputFile)
    LOG('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff' % (imageStdDevs[6], imageStdDevs[7], imageStdDevs[8], imageStdDevs[9], imageStdDevs[10], imageStdDevs[11]), file=trainedModelOutputFile)
    LOG('};', file=trainedModelOutputFile)
    LOG('const float imgGroupDescs[%d][12] = {' % len(imageGroups), file=trainedModelOutputFile)

    imageGroupKeys = sorted(imageGroups.keys())
    
    index = 0
    for imageGroupKey in imageGroupKeys:
        imageGroup = imageGroups[imageGroupKey]
        desc = imageGroup['desc'] 
        LOG('{ // Group %d (key=%d), from %d images' % (index, imageGroupKey, len(imageGroup['members'])), file=trainedModelOutputFile)
        LOG('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff,' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5]), file=trainedModelOutputFile)
        LOG('%12ff, %12ff, %12ff, %12ff, %12ff, %12ff' % (desc[6], desc[7], desc[8], desc[9], desc[10], desc[11]), file=trainedModelOutputFile)
        LOG('},', file=trainedModelOutputFile)
        index += 1
    LOG('};', file=trainedModelOutputFile)
    LOG('', file=trainedModelOutputFile)

    index = 0
    for imageGroupKey in imageGroupKeys:
        imageGroup = imageGroups[imageGroupKey]
        LOG('const int nUnoccupiedCellGroupsForImgGroup%d = %d;' % (index, len(imageGroup['unoccupiedGroups'])), file=trainedModelOutputFile)
        if len(imageGroup['unoccupiedGroups']):
            LOG('const float unoccupiedCellGroupsForImgGroup%d[%d][6] = {' % (index, len(imageGroup['unoccupiedGroups'])), file=trainedModelOutputFile)
            cellIndex = 0
            cellGroupKeys = sorted(imageGroup['unoccupiedGroups'].keys())
            for cellGroupKey in cellGroupKeys:
                cellGroup = imageGroup['unoccupiedGroups'][cellGroupKey]
                desc = cellGroup['desc']
                LOG('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // Group %d (key=%d), from %d cells' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], cellIndex, cellGroupKey, len(cellGroup['members'])), file=trainedModelOutputFile)
                cellIndex += 1
            LOG('};', file=trainedModelOutputFile)
        LOG('const int nOccupiedCellGroupsForImgGroup%d = %d;' % (index, len(imageGroup['occupiedGroups'])), file=trainedModelOutputFile)
        if len(imageGroup['occupiedGroups']):
            LOG('const float occupiedCellGroupsForImgGroup%d[%d][6] = {' % (index, len(imageGroup['occupiedGroups'])), file=trainedModelOutputFile)
            cellIndex = 0
            cellGroupKeys = sorted(imageGroup['occupiedGroups'].keys())
            for cellGroupKey in cellGroupKeys:
                cellGroup = imageGroup['occupiedGroups'][cellGroupKey]
                desc = cellGroup['desc']
                LOG('{%12ff, %12ff, %12ff, %12ff, %12ff, %12ff}, // Group %d (key=%d), from %d cells' % (desc[0], desc[1], desc[2], desc[3], desc[4], desc[5], cellIndex, cellGroupKey, len(cellGroup['members'])), file=trainedModelOutputFile)
                cellIndex += 1
            LOG('};', file=trainedModelOutputFile)
        LOG('', file=trainedModelOutputFile)
        index += 1
    
    # Also write out function to return the appropriate list given a group index. This has to be updated
    # by hand otherwise.
    LOG('\n\n\n\n\n\n\n', file=trainedModelOutputFile)
    LOG('\t\t\t// Get set of cellGroups for this imageGroup', file=trainedModelOutputFile)
    LOG('\t\t\tswitch (imgGroupIndex)', file=trainedModelOutputFile)
    LOG('\t\t\t{', file=trainedModelOutputFile)
    
    index = 0
    for imageGroupKey in imageGroupKeys:
        imageGroup = imageGroups[imageGroupKey]
        LOG('\t\t\tcase %d:' % index, file=trainedModelOutputFile)
        LOG('\t\t\t\tnUnoccupCellGroups = nUnoccupiedCellGroupsForImgGroup%d;' % index, file=trainedModelOutputFile)
        LOG('\t\t\t\tnOccupCellGroups = nOccupiedCellGroupsForImgGroup%d;' % index, file=trainedModelOutputFile)
        
        cellGroup = '0';
        if len(imageGroup['unoccupiedGroups']):
            cellGroup = 'unoccupiedCellGroupsForImgGroup%d' % index
        LOG('\t\t\t\tunoccupGroupDescs = %s;' % cellGroup, file=trainedModelOutputFile)
            
        cellGroup = '0';
        if len(imageGroup['occupiedGroups']):
            cellGroup = 'occupiedCellGroupsForImgGroup%d' % index
        LOG('\t\t\t\toccupGroupDescs = %s;' % cellGroup, file=trainedModelOutputFile)
        
        LOG('\t\t\t\tbreak;', file=trainedModelOutputFile)
        index += 1

    LOG('\t\t\tdefault:', file=trainedModelOutputFile)
    LOG('\t\t\t\tnUnoccupCellGroups = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\tnOccupCellGroups = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\tunoccupGroupDescs = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\toccupGroupDescs = 0;', file=trainedModelOutputFile)
    LOG('\t\t\t\tbreak;', file=trainedModelOutputFile)
    LOG('\t\t\t}', file=trainedModelOutputFile)

@WRAP_FUNCTION
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

@WRAP_FUNCTION
def writeImageGroupsNEW(imageGroups, imageStdDevs):
    robotVisionUtils.pickleItem(imageGroups, robotVisionUtils.IMAGE_GROUPS_PICKLE)
    robotVisionUtils.pickleItem(imageStdDevs, robotVisionUtils.IMAGE_STDDEVS_PICKLE)

    outputFile = open(robotVisionUtils.IMAGE_GROUPS_PICKLE.replace('.p', '.txt'), 'w')
    pprint.pprint(imageGroups, outputFile)
    
    writeImageGroupsCodeNEW(imageGroups, imageStdDevs)

@WRAP_FUNCTION
def writeImageGroupsNEW2(imageGroups, imageStdDevs, cellStdDevs):
    robotVisionUtils.pickleItem(imageGroups, robotVisionUtils.IMAGE_GROUPS_PICKLE)
    robotVisionUtils.pickleItem(imageStdDevs, robotVisionUtils.IMAGE_STDDEVS_PICKLE)
    robotVisionUtils.pickleItem(cellStdDevs, robotVisionUtils.CELL_STDDEVS_PICKLE)

    outputFile = open(robotVisionUtils.IMAGE_GROUPS_PICKLE.replace('.p', '.txt'), 'w')
    pprint.pprint(imageGroups, outputFile)
    
    #writeImageGroupsCodeNEW2(imageGroups, imageStdDevs, cellStdDevs)




