import sys
import os.path
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir))
import math
import time
import pprint
from pdb import set_trace
import robotVisionUtils
import groupImages
import groupBackgroundCells
import groupColours
import utils

LOG = utils.log
WRAP_FUNCTION = utils.wrapFunction

@WRAP_FUNCTION
def testRobotRec(colourGroupsDict, outputFile):
    colourDescFiles = robotVisionUtils.getColourDescriptionFilesInDir(robotVisionUtils.ROBOT_REC_DATA_DIR)
    coloursToUse = robotVisionUtils.readColoursToUse(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, robotVisionUtils.ROBOT_REC_COLOURS_FILE))

    LOG('Test colour model against colour training data:', file=outputFile)
    colourCellsDict = {}
    for colourDescFile in colourDescFiles:
        colourId = int(colourDescFile[14:17])
        if colourId not in coloursToUse:
            LOG('Not testing colour as it isn\'t used: %d' % colourId, file=outputFile)
            continue
        colourCells = robotVisionUtils.loadColourDescriptionsInFile(robotVisionUtils.ROBOT_REC_DATA_DIR, colourDescFile)
        colourCellsDict[colourId] = colourCells

    # Get results
    absFunc = math.fabs
    colourResults = {}
    nWrongTotal = 0
    nCellsTotal = 0
    colourKeys = sorted(colourCellsDict.keys())
    for colour in colourKeys:

        colourCells = colourCellsDict[colour]

        resultsThisColour = []
        colourResults[colour] = resultsThisColour

        for colourCell in colourCells:
            
            colourCellId = colourCell[0] * 1000000 + colourCell[1]

            bestThisDiff = 99999
            bestThisGroupKey = -1
            
            bestOtherDiff = 99999
            bestOtherColour = -1
            bestOtherGroupKey = -1

            for otherColour in colourKeys:
                colourGroups = colourGroupsDict[otherColour]
                
                for colourGroupIndex in range(len(colourGroups)):
                    colourGroup = colourGroups[colourGroupIndex]
                    diff = groupColours.calcColourCellGroupDiff(colourCell, colourGroup['desc'])
                    
                    if colour == otherColour:
                        if diff < bestThisDiff:
                            bestThisDiff = diff
                            bestThisGroupKey = colourGroupIndex
                    else:
                        if diff < bestOtherDiff:
                            bestOtherDiff = diff
                            bestOtherColour = otherColour
                            bestOtherGroupKey = colourGroupIndex
                
            resultsThisColour.append((colourCellId, bestThisDiff, bestThisGroupKey, bestOtherDiff, bestOtherColour, bestOtherGroupKey))

    # Analyse results
    avgDiffs = {}
    for colour in colourKeys:
        resultsThisColour = colourResults[colour]
        
        nCells = len(resultsThisColour)
        
        avgDiffThisColour = sum((x[1] for x in resultsThisColour)) / nCells
        avgDiffOtherColours = sum((x[3] for x in resultsThisColour)) / nCells
        avgDiffs[colour] = avgDiffThisColour
                
        wrongCells = [x for x in resultsThisColour if x[3] <= x[1]]
        nWrong = len(wrongCells)

        LOG('TEST_COLOUR colour=%d avgDiff=%f avgBestOtherDiff=%f nCells=%d nWrong=%d (%f)' % (colour, avgDiffThisColour, avgDiffOtherColours, nCells, nWrong, nWrong/nCells), file=outputFile)
        for otherColour in colourKeys:
            if otherColour == colour:
                continue
            wrongCellsThisColour = [x for x in wrongCells if x[4] == otherColour]
            if wrongCellsThisColour:
                avgThisColourDiff = sum((x[1] for x in wrongCellsThisColour)) / len(wrongCellsThisColour)
                avgOtherColourDiff = sum((x[3] for x in wrongCellsThisColour)) / len(wrongCellsThisColour)
                LOG('    OTHER_COLOUR colour=%d nWrong=%d avgThisColour=%f avgOtherColour=%f' % (otherColour, len(wrongCellsThisColour), avgThisColourDiff, avgOtherColourDiff), file=outputFile)
                
        nCellsTotal += nCells
        nWrongTotal += nWrong
    
    LOG('TEST_ALL_COLOUR nCells=%d nWrong=%d (%f)' % (nCellsTotal, nWrongTotal, nWrongTotal / nCellsTotal), file=outputFile)
    
    LOG('Colour avgs;', file=outputFile)
    colourAvgsKeys = sorted(avgDiffs.keys())
    colourIndex = 0
    for colourAvgsKey in colourAvgsKeys:
        LOG('colour %d: id=%d avg=%f' % (colourIndex, colourAvgsKey, avgDiffs[colourAvgsKey]), file=outputFile)
        colourIndex += 1
    return avgDiffs


@WRAP_FUNCTION
def testRobotRecUsingAvgs(colourGroupsDict, colourAvgs, outputFile):
    colourDescFiles = robotVisionUtils.getColourDescriptionFilesInDir(robotVisionUtils.ROBOT_REC_DATA_DIR)
    coloursToUse = robotVisionUtils.readColoursToUse(os.path.join(robotVisionUtils.ROBOT_REC_DATA_DIR, robotVisionUtils.ROBOT_REC_COLOURS_FILE))

    LOG('Test colour model against colour training data USING AVGS:', file=outputFile)
    colourCellsDict = {}
    for colourDescFile in colourDescFiles:
        colourId = int(colourDescFile[14:17])
        if colourId not in coloursToUse:
            LOG('Not testing colour as it isn\'t used: %d' % colourId, file=outputFile)
            continue
        colourCells = robotVisionUtils.loadColourDescriptionsInFile(robotVisionUtils.ROBOT_REC_DATA_DIR, colourDescFile)
        colourCellsDict[colourId] = colourCells
    
    # Get results
    absFunc = math.fabs
    colourResults = {}
    nWrongTotal = 0
    nCellsTotal = 0
    colourKeys = sorted(colourCellsDict.keys())
    for colour in colourKeys:

        colourCells = colourCellsDict[colour]

        resultsThisColour = []
        colourResults[colour] = resultsThisColour

        for colourCell in colourCells:
            
            colourCellId = colourCell[0] * 1000000 + colourCell[1]

            bestThisDiff = 99999
            bestThisGroupKey = -1
            
            bestOtherDiff = 99999
            bestOtherColour = -1
            bestOtherGroupKey = -1

            for otherColour in colourKeys:
                colourGroups = colourGroupsDict[otherColour]
                
                for colourGroupIndex in range(len(colourGroups)):
                
                    colourGroup = colourGroups[colourGroupIndex]
                    diff = groupColours.calcColourCellGroupDiff(colourCell, colourGroup['desc'])
                    normdDiff = diff / colourAvgs[otherColour]
                    
                    if colour == otherColour:
                        if normdDiff < bestThisDiff:
                            bestThisDiff = normdDiff
                            bestThisGroupKey = colourGroupIndex
                    else:
                        if normdDiff < bestOtherDiff:
                            bestOtherDiff = normdDiff
                            bestOtherColour = otherColour
                            bestOtherGroupKey = colourGroupIndex
                
            resultsThisColour.append((colourCellId, bestThisDiff, bestThisGroupKey, bestOtherDiff, bestOtherColour, bestOtherGroupKey))

    # Analyse results
    for colour in colourKeys:
        resultsThisColour = colourResults[colour]

        nCells = len(resultsThisColour)
        
        avgDiffThisColour = sum((x[1] for x in resultsThisColour)) / nCells
        avgDiffOtherColours = sum((x[3] for x in resultsThisColour)) / nCells

        wrongCells = [x for x in resultsThisColour if x[3] <= x[1]]
        nWrong = len(wrongCells)

        LOG('TEST_COLOUR colour=%d avgDiff=%f avgBestOtherDiff=%f nCells=%d nWrong=%d (%f)' % (colour, avgDiffThisColour, avgDiffOtherColours, nCells, nWrong, nWrong/nCells), file=outputFile)
        for otherColour in colourKeys:
            if otherColour == colour:
                continue
            wrongCellsThisColour = [x for x in wrongCells if x[4] == otherColour]
            if wrongCellsThisColour:
                avgThisColourDiff = sum((x[1] for x in wrongCellsThisColour)) / len(wrongCellsThisColour)
                avgOtherColourDiff = sum((x[3] for x in wrongCellsThisColour)) / len(wrongCellsThisColour)
                LOG('    OTHER_COLOUR colour=%d nWrong=%d avgThisColour=%f avgOtherColour=%f' % (otherColour, len(wrongCellsThisColour), avgThisColourDiff, avgOtherColourDiff), file=outputFile)
                
        nCellsTotal += nCells
        nWrongTotal += nWrong
        

    LOG('TEST_ALL_COLOUR nCells=%d nWrong=%d (%f)' % (nCellsTotal, nWrongTotal, nWrongTotal / nCellsTotal), file=outputFile)

    nDiffsTooHighTotal = 0
    for colour in colourKeys:
        resultsThisColour = colourResults[colour]
        
        problematicCells = [x for x in resultsThisColour if x[1] >= (robotVisionUtils.COLOUR_TESTING_THRESHOLD * 0.9)]
        nDiffsTooHigh = len(problematicCells)
        nDiffsTooHighTotal += nDiffsTooHigh
        
        LOG('DIFFS_TOO_HIGH colour=%d n=%d' % (colour, nDiffsTooHigh), file=outputFile)
        for problematicCell in problematicCells:
            (colourCellId, bestThisDiff, bestThisGroupKey, bestOtherDiff, bestOtherColour, bestOtherGroupKey) = problematicCell
            LOG('cell=%d diff=%f groupKey=%d bestOtherDiff=%f otherColour=%d otherGroupkey=%d' % (colourCellId, bestThisDiff, bestThisGroupKey, bestOtherDiff, bestOtherColour, bestOtherGroupKey), file=outputFile)
    
    LOG('DIFFS_TOO_HIGH_TOTAL n=%d' % (nDiffsTooHighTotal), file=outputFile)

    
@WRAP_FUNCTION
def testRobotRecAgainstBackgroundImages(colourGroupsDict, colourAvgsDict, outputFile):

    cellDescFiles = robotVisionUtils.getCellDescriptionFilesInDir(robotVisionUtils.OBST_REC_DATA_DIR)
    
    colours = sorted(colourGroupsDict.keys())
    
    results = []
    compFunc = groupBackgroundCells.compareCells

    LOG('Test colour model against background training data:', file=outputFile)

    #set_trace()
    for cellDescFile in cellDescFiles:

        imageNum = int(cellDescFile[12:15])

        # Load cellDescs for testing; load all descs, including those marked as unknown, and
        # record isOccupied flag with each desc
        cellDescs = robotVisionUtils.loadCellDescriptionsInFile(robotVisionUtils.OBST_REC_DATA_DIR, cellDescFile, isForTraining=0)
        cellIndex = 0
        for isOccupied, cellDesc in cellDescs:
        
            minDiff = 99999
            minColour = -1
            minColourGroup = -1
            
            for colour in colours:
                colourGroups = colourGroupsDict[colour]
                colourAvg = colourAvgsDict[colour]
                
                minDiff = 99999
                minNormdDiff = 99999
                minColourGroup = -1
                for colourGroupIndex in range(len(colourGroups)):
                    colourGroup = colourGroups[colourGroupIndex]
                    diff = compFunc(cellDesc, colourGroup['desc'])
                    normdDiff = diff / colourAvg

                    if normdDiff < minNormdDiff:
                        minDiff = diff
                        minNormdDiff = normdDiff
                        minColourGroup = colourGroupIndex

                if minNormdDiff < (robotVisionUtils.COLOUR_TESTING_THRESHOLD * 1.3):
                    result = (cellDescFile, cellIndex, isOccupied, colour, colourGroupIndex, minDiff, colourAvg, minNormdDiff)
                    results.append(result)
            
            #set_trace()

            cellIndex += 1

    for result in results:
        (file, cell, isOccupied, colourId, colourGroup, diff, colourAvg, normdDiff) = result
        LOG('file=%s cell=%3d isOccupied=%d colourId=%3d colourGroup=%3d diff=%f colourAvg=%f normdDiff=%f' % (file, cell, isOccupied, colourId, colourGroup, diff, colourAvg, normdDiff), file=outputFile)
    LOG('N_PROBLEMATIC_CELLS=%d' % len(results), file=outputFile)
    


            