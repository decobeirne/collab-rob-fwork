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
import utils

LOG = utils.log
WRAP_FUNCTION = utils.wrapFunction

@WRAP_FUNCTION
def testObstRec(imageGroups, imageStdDevs, outputFile):
    cellDescFiles = robotVisionUtils.getCellDescriptionFilesInDir(robotVisionUtils.OBST_REC_DATA_DIR)

    absFunc = math.fabs
    compFunc = groupBackgroundCells.compareCells
    nCells = 0
    sumCorrectDiffs = 0
    nCorrectDiffs = 0
    incorrectCells = {}
    for cellDescFile in cellDescFiles:
        incorrectCellsThisImage = []
        incorrectCells[cellDescFile] = incorrectCellsThisImage

        imageNum = int(cellDescFile[12:15])
        imageDescFile = 'imgFeatures%03d.txt' % imageNum
        imageDesc = robotVisionUtils.loadImageDescriptionInFile(robotVisionUtils.OBST_REC_DATA_DIR, imageDescFile)

        # Load cellDescs for testing; load all descs, including those marked as unknown, and
        # record isOccupied flag with each desc
        cellDescs = robotVisionUtils.loadCellDescriptionsInFile(robotVisionUtils.OBST_REC_DATA_DIR, cellDescFile, isForTraining=0)
        
        # The following should mirror exactly the algorithm in ObstacleRecognitionModel_calcObstacleGroupScores
        camScoreGrid = []
        for isOccupied, cellDesc in cellDescs:
            bestUnoccupMatch = 99999
            bestUnoccupImage = -1
            bestUnoccupCell = -1
            bestOccupMatch = 99999
            bestOccupImage = -1
            bestOccupCell = -1
            
            imageGroupeKeys = sorted(imageGroups.keys())
            for imageGroupKey in imageGroupeKeys:
                imageGroup = imageGroups[imageGroupKey]
                imageGroupDiff = groupImages.compareImageDescsNEW(imageDesc, imageGroup['desc'], imageStdDevs)
                
                # Unoccupied groups
                cellGroupKeys = sorted(imageGroup['unoccupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['unoccupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestUnoccupMatch:
                        bestUnoccupMatch = diff
                        bestUnoccupImage = imageGroupKey
                        bestUnoccupCell = cellGroupKey

                # Occupied groups
                cellGroupKeys = sorted(imageGroup['occupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['occupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestOccupMatch:
                        bestOccupMatch = diff
                        bestOccupImage = imageGroupKey
                        bestOccupCell = cellGroupKey
            
            camScoreGrid.append((isOccupied, (bestUnoccupMatch, bestUnoccupImage, bestUnoccupCell), (bestOccupMatch, bestOccupImage, bestOccupCell)))
        
        # This should match ObstacleRecognition_estimateGridOccupancy
        nCells += len(camScoreGrid)
        for cellIndex in range(len(camScoreGrid)):
            isOccupiedFlag, bestUnoccup, bestOccup = camScoreGrid[cellIndex]
            
            # TODO: work on heuristic wrt confidence of estimate
            isOccupiedEst = bestOccup[0] < bestUnoccup[0]
            if isOccupiedEst == isOccupiedFlag:
                diff = absFunc(bestOccup[0] - bestUnoccup[0])
                sumCorrectDiffs += diff
                nCorrectDiffs += 1
            if isOccupiedEst != isOccupiedFlag and isOccupiedFlag != 2:
                incorrectCellsThisImage.append((cellIndex, camScoreGrid[cellIndex]))
    
    LOG('Total cells %d correct %d (not including cells marked as unknown in training) avg correct diff %f' % (nCells, nCorrectDiffs, sumCorrectDiffs / nCorrectDiffs), file=outputFile)
    LOG('Incorrect cells:', file=outputFile)
    LOG('Error tuple=(cellIndex, (isOccupied, (unoccupMatch, unoccupImg, unoccupCell), (occupMatch, occupImg, occupCell)))', file=outputFile)
    nIncorrectTotal = 0
    for cellDescFile, incorrectCellList in incorrectCells.items():
        nIncorrectThisImage = len(incorrectCellList)
        nIncorrectTotal += nIncorrectThisImage
        LOG('%d incorrect cells in %s' % (nIncorrectThisImage, cellDescFile), file=outputFile)
        for incorrectCell in incorrectCellList:
            LOG(repr(incorrectCell), file=outputFile)
    LOG('%d incorrect in total' % nIncorrectTotal, file=outputFile)

def compareFlagToNeighbours(cellIndex, isOccupiedFlag, camScoreGrid):
    oppositeFlag = (1,0)[isOccupiedFlag]

    gridX = robotVisionUtils.OCCUP_GRID_X_DIMS
    gridY = robotVisionUtils.OCCUP_GRID_Y_DIMS
    cellI = cellIndex % gridX
    cellJ = cellIndex // gridX

    for nbourJ in range(max(0, cellJ - 1), min(gridY, cellJ + 2)):
        for nbourI in range(max(0, cellI - 1), min(gridX, cellI + 2)):
            if nbourI == cellI and nbourJ == cellJ:
                continue
            nbourIndex = nbourI + nbourJ * gridX
            if camScoreGrid[nbourIndex][0] != oppositeFlag:
                return 0
    # Return value should be considered as 'isAnAnomaly'
    return 1

@WRAP_FUNCTION
def testObstRecNEW(imageGroups, imageStdDevs, outputFile):
    cellDescFiles = robotVisionUtils.getCellDescriptionFilesInDir(robotVisionUtils.OBST_REC_DATA_DIR)

    absFunc = math.fabs
    compFunc = groupBackgroundCells.compareCells
    
    confidenceThreshold = robotVisionUtils.OCCUPANCY_CONFIDENCE_THRESHOLD

    nCellsFlags = {'nUnoccupied':0, 'nOccupied':0, 'nUnknown':0}
    nCellsEsts = {'nUnoccupied':0, 'nOccupied':0, 'nUnknown':0}
    confidentUnknownCells = []  # The est is confident, but the cell had been flagged as unknown
    confidentCorrectCells = []
    confidentIncorrectCells = []
    notConfidentUnknownCells = []  # The est is not confident, and the cell was marked as unknown
    notConfidentCorrectCells = []
    notConfidentIncorrectCells = []
    nCellsTotal = 0
    
    for cellDescFile in cellDescFiles:

        imageNum = int(cellDescFile[12:15])
        imageDescFile = 'imgFeatures%03d.txt' % imageNum
        imageDesc = robotVisionUtils.loadImageDescriptionInFile(robotVisionUtils.OBST_REC_DATA_DIR, imageDescFile)

        # Load cellDescs for testing; load all descs, including those marked as unknown, and
        # record isOccupied flag with each desc
        cellDescs = robotVisionUtils.loadCellDescriptionsInFile(robotVisionUtils.OBST_REC_DATA_DIR, cellDescFile, isForTraining=0)
        
        # The following should mirror exactly the algorithm in ObstacleRecognitionModel_calcObstacleGroupScores
        camScoreGrid = []
        for isOccupied, cellDesc in cellDescs:
            bestUnoccupMatch = 99999
            bestUnoccupImage = -1
            bestUnoccupCell = -1
            bestOccupMatch = 99999
            bestOccupImage = -1
            bestOccupCell = -1
            
            imageGroupeKeys = sorted(imageGroups.keys())
            for imageGroupKey in imageGroupeKeys:
                imageGroup = imageGroups[imageGroupKey]
                imageGroupDiff = groupImages.compareImageDescsNEW(imageDesc, imageGroup['desc'], imageStdDevs)
                
                # Unoccupied groups
                cellGroupKeys = sorted(imageGroup['unoccupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['unoccupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestUnoccupMatch:
                        bestUnoccupMatch = diff
                        bestUnoccupImage = imageGroupKey
                        bestUnoccupCell = cellGroupKey

                # Occupied groups
                cellGroupKeys = sorted(imageGroup['occupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['occupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestOccupMatch:
                        bestOccupMatch = diff
                        bestOccupImage = imageGroupKey
                        bestOccupCell = cellGroupKey
            
            camScoreGrid.append((isOccupied, (bestUnoccupMatch, bestUnoccupImage, bestUnoccupCell), (bestOccupMatch, bestOccupImage, bestOccupCell)))
        
        # Hysteresis function - can remove this if not benefitial
        if 1:
            for cellIndex in range(len(camScoreGrid)):
                isOccupiedFlag, bestUnoccup, bestOccup = camScoreGrid[cellIndex]
                if isOccupiedFlag in (0,1):
                    isAnomaly = compareFlagToNeighbours(cellIndex, isOccupiedFlag, camScoreGrid)
                    if isAnomaly:
                        set_trace()
                        LOG('Setting cell %3d from %d to 2 in %s' % (cellIndex, isOccupiedFlag, cellDescFile), file=outputFile)
                        newTuple = (2, bestUnoccup, bestOccup)  # 2=unknown
                        camScoreGrid[cellIndex] = newTuple
        
        # This should match ObstacleRecognition_estimateGridOccupancy
        for cellIndex in range(len(camScoreGrid)):
            isOccupiedFlag, bestUnoccup, bestOccup = camScoreGrid[cellIndex]
            
            isConfidentFlag = (isOccupiedFlag != 2)
            if isOccupiedFlag == 0:
                nCellsFlags['nUnoccupied'] += 1
            elif isOccupiedFlag == 1:
                nCellsFlags['nOccupied'] += 1
            else:
                nCellsFlags['nUnknown'] += 1
                
            isOccupiedEst = bestOccup[0] < bestUnoccup[0]
            cellRatio = (bestUnoccup[0] / bestOccup[0]) if isOccupiedEst else (bestOccup[0] / bestUnoccup[0])  # Take ratio of larger over smaller (est) value
            isConfidentEst = cellRatio > confidenceThreshold
            isCorrect = isOccupiedEst == isOccupiedFlag
            if isConfidentEst:
                if not isConfidentFlag:
                    confidentUnknownCells.append(cellRatio)
                elif isCorrect:
                    confidentCorrectCells.append(cellRatio)
                else:
                    confidentIncorrectCells.append(cellRatio)
            else:
                if not isConfidentFlag:
                    notConfidentUnknownCells.append(cellRatio)
                elif isCorrect:
                    notConfidentCorrectCells.append(cellRatio)
                else:
                    notConfidentIncorrectCells.append(cellRatio)
                if 1:
                    larger = max(bestOccup[0], bestUnoccup[0])
                    smaller = min(bestOccup[0], bestUnoccup[0])
                    LOG('%f/%f=%f' % (larger, smaller, larger / smaller), file=outputFile)
            
            
            if not isConfidentEst:
                nCellsEsts['nUnknown'] += 1
            elif isOccupiedEst:
                nCellsEsts['nOccupied'] += 1
            else:
                nCellsEsts['nUnoccupied'] += 1
        
            nCellsTotal += 1

    LOG('Tested %d cells' % nCellsTotal, file=outputFile)
    LOG('actual cells: nUnoccupied:%d nOccupied:%d nUnknown:%d (%d)' % (nCellsFlags['nUnoccupied'], nCellsFlags['nOccupied'], nCellsFlags['nUnknown'], (nCellsFlags['nUnoccupied'] + nCellsFlags['nOccupied'] + nCellsFlags['nUnknown'])), file=outputFile)
    LOG('estd cells:   nUnoccupied:%d nOccupied:%d nUnknown:%d (%d)' % (nCellsEsts['nUnoccupied'], nCellsEsts['nOccupied'], nCellsEsts['nUnknown'], (nCellsEsts['nUnoccupied'] + nCellsEsts['nOccupied'] + nCellsEsts['nUnknown'])), file=outputFile)
    
    def __rat(val1, val2):
        if val2:
            return val1 / val2
        else:
            return 0
    
    nConfidentCorrect = len(confidentCorrectCells)
    nConfidentUnknown = len(confidentUnknownCells)
    nConfidentIncorrect = len(confidentIncorrectCells)
    nConfident = nConfidentCorrect + nConfidentUnknown + nConfidentIncorrect
    LOG('confident: nConfident:%d nCorrect:%d (%f) nIncorrect:%d (%f) unKnown:%d (%f)' % (nConfident, nConfidentCorrect, __rat(nConfidentCorrect, nConfident), nConfidentIncorrect, __rat(nConfidentIncorrect, nConfident), nConfidentUnknown, __rat(nConfidentUnknown, nConfident)), file=outputFile)
    confidentCorrectOccupCells = [x for x in confidentCorrectCells if x < 1]
    confidentCorrectUnoccupCells = [x for x in confidentCorrectCells if x not in confidentCorrectOccupCells]
    nConfidentCorrectOccup = len(confidentCorrectOccupCells)
    nConfidentCorrectUnoccup = len(confidentCorrectUnoccupCells)
    LOG('confident correct: nUnoccup:%d nOccup:%d' % (nConfidentCorrectUnoccup, nConfidentCorrectOccup), file=outputFile)
    confidentUnknownOccupCells = [x for x in confidentUnknownCells if x < 1]
    confidentUnknownUnoccupCells = [x for x in confidentUnknownCells if x not in confidentUnknownOccupCells]
    nConfidentUnknownOccup = len(confidentUnknownOccupCells)
    nConfidentUnknownUnoccup = len(confidentUnknownUnoccupCells)
    LOG('confident unknown: nUnoccup:%d nOccup:%d' % (nConfidentUnknownUnoccup, nConfidentUnknownOccup), file=outputFile)
    confidentIncorrectOccupCells = [x for x in confidentIncorrectCells if x < 1]
    confidentIncorrectUnoccupCells = [x for x in confidentIncorrectCells if x not in confidentIncorrectOccupCells]
    nConfidentIncorrectOccup = len(confidentIncorrectOccupCells)
    nConfidentIncorrectUnoccup = len(confidentIncorrectUnoccupCells)
    LOG('confident incorrect: nUnoccup:%d nOccup:%d' % (nConfidentIncorrectUnoccup, nConfidentIncorrectOccup), file=outputFile)
    
    nNotConfidentCorrect = len(notConfidentCorrectCells)
    nNotConfidentUnknown = len(notConfidentUnknownCells)
    nNotConfidentIncorrect = len(notConfidentIncorrectCells)
    nNotConfident = nNotConfidentCorrect + nNotConfidentUnknown + nNotConfidentIncorrect
    LOG('notConfident: nNotConfident:%d nCorrect:%d (%f) nIncorrect:%d (%f) nUnknown:%d (%f)' % (nNotConfident, nNotConfidentCorrect, __rat(nNotConfidentCorrect, nConfident), nNotConfidentIncorrect, __rat(nNotConfidentIncorrect, nNotConfident), nNotConfidentUnknown, __rat(nNotConfidentUnknown, nNotConfident)), file=outputFile)
    notConfidentCorrectOccupCells = [x for x in notConfidentCorrectCells if x < 1]
    notConfidentCorrectUnoccupCells = [x for x in notConfidentCorrectCells if x not in notConfidentCorrectOccupCells]
    nNotConfidentCorrectOccup = len(notConfidentCorrectOccupCells)
    nNotConfidentCorrectUnoccup = len(notConfidentCorrectUnoccupCells)
    LOG('notConfident correct: nUnoccup:%d nOccup:%d' % (nNotConfidentCorrectUnoccup, nNotConfidentCorrectOccup), file=outputFile)
    notConfidentUnknownOccupCells = [x for x in notConfidentUnknownCells if x < 1]
    notConfidentUnknownUnoccupCells = [x for x in notConfidentUnknownCells if x not in notConfidentUnknownOccupCells]
    nNotConfidentUnknownOccup = len(notConfidentUnknownOccupCells)
    nNotConfidentUnknownUnoccup = len(notConfidentUnknownUnoccupCells)
    LOG('notConfident unknown: nUnoccup:%d nOccup:%d' % (nNotConfidentUnknownUnoccup, nNotConfidentUnknownOccup), file=outputFile)
    notConfidentIncorrectOccupCells = [x for x in notConfidentIncorrectCells if x < 1]
    notConfidentIncorrectUnoccupCells = [x for x in notConfidentIncorrectCells if x not in notConfidentIncorrectOccupCells]
    nNotConfidentIncorrectOccup = len(notConfidentIncorrectOccupCells)
    nNotConfidentIncorrectUnoccup = len(notConfidentIncorrectUnoccupCells)
    LOG('notConfident incorrect: nUnoccup:%d nOccup:%d' % (nNotConfidentIncorrectUnoccup, nNotConfidentIncorrectOccup), file=outputFile)

    
def applySmoothing(cellX, cellY, gridX, gridY, cellIndex, camScoreGrid, cellDescs):
    compFunc = groupBackgroundCells.compareCells
    
    nbourDiffThreshold = 60
    nbourDiffThresholdInv = 1 / nbourDiffThreshold
    adjustmentCoefficient = 0.2
    
    #set_trace()
    isOccupiedFlag, bestUnoccup, bestOccup = camScoreGrid[cellIndex]
    isOccupiedFlag, cellDesc = cellDescs[cellIndex]
    
    # Get nbours
    nbourAdjustments = []
    for nbourI in range(max(0, cellX - 1), min(gridX, cellX + 2)):
        for nbourJ in range(max(0, cellY - 1), min(gridY, cellY + 2)):
            if nbourI == cellX and nbourJ == cellY:
                continue
            nbourOccupiedFlag, nbourBestUnoccup, nbourBestOccup = camScoreGrid[nbourI + nbourJ * gridX]
            
            nbourIsOccupied, nbourDesc = cellDescs[nbourI + nbourJ * gridX]
            nbourDiff = compFunc(cellDesc, nbourDesc)
            if nbourDiff < nbourDiffThreshold:
                nbourScore = 1 - (nbourDiff * nbourDiffThresholdInv)
                nbourRatio = nbourBestOccup[0] / nbourBestUnoccup[0]
                nbourAdjustments.append((nbourScore, nbourRatio))
    
    #set_trace()
    # Apply adjustment to own ratio based on nbours
    adjustments = []
    ownRatio = bestOccup[0] / bestUnoccup[0]
    for nbourAdjustment in nbourAdjustments:
        ratioDiff = nbourAdjustment[1] - ownRatio
        adjustment = ratioDiff * nbourScore * adjustmentCoefficient
        adjustments.append(adjustment)

    finalAdjustment = sum(adjustments)
    finalRatio = ownRatio + finalAdjustment
    return finalRatio

@WRAP_FUNCTION
def testObstRecWithSmoothing(imageGroups, imageStdDevs, outputFile):
    cellDescFiles = robotVisionUtils.getCellDescriptionFilesInDir(robotVisionUtils.OBST_REC_DATA_DIR)

    absFunc = math.fabs
    compFunc = groupBackgroundCells.compareCells
    nCells = 0
    sumCorrectDiffs = 0
    nCorrectDiffs = 0
    incorrectCells = {}
    for cellDescFile in cellDescFiles:
        incorrectCellsThisImage = []
        incorrectCells[cellDescFile] = incorrectCellsThisImage

        imageNum = int(cellDescFile[12:15])
        imageDescFile = 'imgFeatures%03d.txt' % imageNum
        imageDesc = robotVisionUtils.loadImageDescriptionInFile(robotVisionUtils.OBST_REC_DATA_DIR, imageDescFile)

        # Load cellDescs for testing; load all descs, including those marked as unknown, and
        # record isOccupied flag with each desc
        cellDescs = robotVisionUtils.loadCellDescriptionsInFile(robotVisionUtils.OBST_REC_DATA_DIR, cellDescFile, isForTraining=0)
        
        # The following should mirror exactly the algorithm in ObstacleRecognitionModel_calcObstacleGroupScores
        camScoreGrid = []
        for isOccupied, cellDesc in cellDescs:
            bestUnoccupMatch = 99999
            bestUnoccupImage = -1
            bestUnoccupCell = -1
            bestOccupMatch = 99999
            bestOccupImage = -1
            bestOccupCell = -1
            
            imageGroupeKeys = sorted(imageGroups.keys())
            for imageGroupKey in imageGroupeKeys:
                imageGroup = imageGroups[imageGroupKey]
                imageGroupDiff = groupImages.compareImageDescsNEW(imageDesc, imageGroup['desc'], imageStdDevs)
                
                # Unoccupied groups
                cellGroupKeys = sorted(imageGroup['unoccupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['unoccupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestUnoccupMatch:
                        bestUnoccupMatch = diff
                        bestUnoccupImage = imageGroupKey
                        bestUnoccupCell = cellGroupKey

                # Occupied groups
                cellGroupKeys = sorted(imageGroup['occupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['occupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestOccupMatch:
                        bestOccupMatch = diff
                        bestOccupImage = imageGroupKey
                        bestOccupCell = cellGroupKey
            
            camScoreGrid.append((isOccupied, (bestUnoccupMatch, bestUnoccupImage, bestUnoccupCell), (bestOccupMatch, bestOccupImage, bestOccupCell)))
        
        # This should match ObstacleRecognition_estimateGridOccupancy
        # Changed >>>>>
        #set_trace()
        nCells += len(camScoreGrid)
        gridX = 23
        assert nCells % gridX == 0
        gridY = nCells // 23
        smoothGrid = []
        cellIndex = 0
        for cellY in range(gridY):
            for cellX in range(gridX):
                isOccupiedFlag, bestUnoccup, bestOccup = camScoreGrid[cellIndex]
                smoothResult = applySmoothing(cellX, cellY, gridX, gridY, cellIndex, camScoreGrid, cellDescs)
                smoothGrid.append((isOccupiedFlag, smoothResult))
                cellIndex += 1
        #set_trace()
        for cellIndex in range(len(smoothGrid)):
            isOccupiedFlag, ratio = smoothGrid[cellIndex]
            
            # TODO: work on heuristic wrt confidence of estimate
            isOccupiedEst = ratio < 1
            if isOccupiedEst == isOccupiedFlag:
                diff = absFunc(bestOccup[0] - bestUnoccup[0])
                sumCorrectDiffs += diff
                nCorrectDiffs += 1
            if isOccupiedEst != isOccupiedFlag and isOccupiedFlag != 2:
                incorrectCellsThisImage.append((cellIndex, camScoreGrid[cellIndex]))
        
        # <<<<<
    
    LOG('Total cells %d correct %d (not including cells marked as unknown in training) avg correct diff %f' % (nCells, nCorrectDiffs, sumCorrectDiffs / nCorrectDiffs), file=outputFile)
    LOG('Incorrect cells:', file=outputFile)
    LOG('Error tuple=(cellIndex, (isOccupied, (unoccupMatch, unoccupImg, unoccupCell), (occupMatch, occupImg, occupCell)))', file=outputFile)
    nIncorrectTotal = 0
    for cellDescFile, incorrectCellList in incorrectCells.items():
        nIncorrectThisImage = len(incorrectCellList)
        nIncorrectTotal += nIncorrectThisImage
        LOG('%d incorrect cells in %s' % (nIncorrectThisImage, cellDescFile), file=outputFile)
        for incorrectCell in incorrectCellList:
            LOG(repr(incorrectCell), file=outputFile)
    LOG('%d incorrect in total' % nIncorrectTotal, file=outputFile)

@WRAP_FUNCTION
def testObstRecWithSmoothingNEW(imageGroups, imageStdDevs, outputFile):
    cellDescFiles = robotVisionUtils.getCellDescriptionFilesInDir(robotVisionUtils.OBST_REC_DATA_DIR)

    absFunc = math.fabs
    compFunc = groupBackgroundCells.compareCells
    
    confidenceThreshold = 0.4
    
    nCellsFlags = {'nUnoccupied':0, 'nOccupied':0, 'nUnknown':0}
    nCellsEsts = {'nUnoccupied':0, 'nOccupied':0, 'nUnknown':0}
    confidentUnknownCells = []  # The est is confident, but the cell had been flagged as unknown
    confidentCorrectCells = []
    confidentIncorrectCells = []
    notConfidentUnknownCells = []  # The est is not confident, and the cell was marked as unknown
    notConfidentCorrectCells = []
    notConfidentIncorrectCells = []
    nCellsTotal = 0
    
    LOG('\n\nObstacle recognition function using smoothing...\n\n', file=outputFile)
    
    for cellDescFile in cellDescFiles:

        imageNum = int(cellDescFile[12:15])
        imageDescFile = 'imgFeatures%03d.txt' % imageNum
        imageDesc = robotVisionUtils.loadImageDescriptionInFile(robotVisionUtils.OBST_REC_DATA_DIR, imageDescFile)

        # Load cellDescs for testing; load all descs, including those marked as unknown, and
        # record isOccupied flag with each desc
        cellDescs = robotVisionUtils.loadCellDescriptionsInFile(robotVisionUtils.OBST_REC_DATA_DIR, cellDescFile, isForTraining=0)
        
        # The following should mirror exactly the algorithm in ObstacleRecognitionModel_calcObstacleGroupScores
        camScoreGrid = []
        for isOccupied, cellDesc in cellDescs:
            bestUnoccupMatch = 99999
            bestUnoccupImage = -1
            bestUnoccupCell = -1
            bestOccupMatch = 99999
            bestOccupImage = -1
            bestOccupCell = -1
            
            imageGroupeKeys = sorted(imageGroups.keys())
            for imageGroupKey in imageGroupeKeys:
                imageGroup = imageGroups[imageGroupKey]
                imageGroupDiff = groupImages.compareImageDescsNEW(imageDesc, imageGroup['desc'], imageStdDevs)
                
                # Unoccupied groups
                cellGroupKeys = sorted(imageGroup['unoccupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['unoccupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestUnoccupMatch:
                        bestUnoccupMatch = diff
                        bestUnoccupImage = imageGroupKey
                        bestUnoccupCell = cellGroupKey

                # Occupied groups
                cellGroupKeys = sorted(imageGroup['occupiedGroups'].keys())
                for cellGroupKey in cellGroupKeys:
                    cellGroup = imageGroup['occupiedGroups'][cellGroupKey]
                    cellDiff = compFunc(cellDesc, cellGroup['desc'])
                    diff = cellDiff * robotVisionUtils.CELL_DIFF_COEFF + imageGroupDiff * robotVisionUtils.IMAGE_DIFF_COEFF
                    if diff < bestOccupMatch:
                        bestOccupMatch = diff
                        bestOccupImage = imageGroupKey
                        bestOccupCell = cellGroupKey
            
            camScoreGrid.append((isOccupied, (bestUnoccupMatch, bestUnoccupImage, bestUnoccupCell), (bestOccupMatch, bestOccupImage, bestOccupCell)))
        
        # This should match ObstacleRecognition_estimateGridOccupancy
        # Changed >>>>>
        nCells = len(camScoreGrid)
        nCellsTotal += nCells
        gridX = 23
        assert nCells % gridX == 0
        gridY = nCells // 23
        
        cellIndex = 0
        smoothScoreGrid = []
        for cellY in range(gridY):
            for cellX in range(gridX):
                isOccupiedFlag, bestUnoccup, bestOccup = camScoreGrid[cellIndex]
                smoothResult = applySmoothing(cellX, cellY, gridX, gridY, cellIndex, camScoreGrid, cellDescs)
                smoothScoreGrid.append((isOccupiedFlag, smoothResult))
                cellIndex += 1

        for cellIndex in range(len(smoothScoreGrid)):
            isOccupiedFlag, cellRatio = smoothScoreGrid[cellIndex]
            
            isConfidentFlag = (isOccupiedFlag != 2)
            if isOccupiedFlag == 0:
                nCellsFlags['nUnoccupied'] += 1
            elif isOccupiedFlag == 1:
                nCellsFlags['nOccupied'] += 1
            else:
                nCellsFlags['nUnknown'] += 1
            
            isOccupiedEst = cellRatio < 1.0
            isConfidentEst = absFunc(cellRatio - 1.0) > confidenceThreshold
            isCorrect = isOccupiedEst == isOccupiedFlag
            if isConfidentEst:
                if not isConfidentFlag:
                    confidentUnknownCells.append(cellRatio)
                elif isCorrect:
                    confidentCorrectCells.append(cellRatio)
                else:
                    confidentIncorrectCells.append(cellRatio)
            else:
                if not isConfidentFlag:
                    notConfidentUnknownCells.append(cellRatio)
                elif isCorrect:
                    notConfidentCorrectCells.append(cellRatio)
                else:
                    notConfidentIncorrectCells.append(cellRatio)
            
            if not isConfidentEst:
                nCellsEsts['nUnknown'] += 1
            elif isOccupiedEst:
                nCellsEsts['nOccupied'] += 1
            else:
                nCellsEsts['nUnoccupied'] += 1
        
            cellIndex += 1
        # <<<<<

    LOG('Tested %d cells' % nCellsTotal, file=outputFile)
    LOG('actual cells: nUnoccupied:%d nOccupied:%d nUnknown:%d (%d)' % (nCellsFlags['nUnoccupied'], nCellsFlags['nOccupied'], nCellsFlags['nUnknown'], (nCellsFlags['nUnoccupied'] + nCellsFlags['nOccupied'] + nCellsFlags['nUnknown'])), file=outputFile)
    LOG('estd cells:   nUnoccupied:%d nOccupied:%d nUnknown:%d (%d)' % (nCellsEsts['nUnoccupied'], nCellsEsts['nOccupied'], nCellsEsts['nUnknown'], (nCellsEsts['nUnoccupied'] + nCellsEsts['nOccupied'] + nCellsEsts['nUnknown'])), file=outputFile)
    
    nConfidentCorrect = len(confidentCorrectCells)
    nConfidentUnknown = len(confidentUnknownCells)
    nConfidentIncorrect = len(confidentIncorrectCells)
    nConfident = nConfidentCorrect + nConfidentUnknown + nConfidentIncorrect
    LOG('confident: nConfident:%d nCorrect:%d (%f) nIncorrect:%d (%f) unKnown:%d (%f)' % (nConfident, nConfidentCorrect, nConfidentCorrect / nConfident, nConfidentIncorrect, nConfidentIncorrect / nConfident, nConfidentUnknown, nConfidentUnknown / nConfident), file=outputFile)
    confidentCorrectOccupCells = [x for x in confidentCorrectCells if x < 1]
    confidentCorrectUnoccupCells = [x for x in confidentCorrectCells if x not in confidentCorrectOccupCells]
    nConfidentCorrectOccup = len(confidentCorrectOccupCells)
    nConfidentCorrectUnoccup = len(confidentCorrectUnoccupCells)
    LOG('confident correct: nUnoccup:%d nOccup:%d' % (nConfidentCorrectUnoccup, nConfidentCorrectOccup), file=outputFile)
    confidentUnknownOccupCells = [x for x in confidentUnknownCells if x < 1]
    confidentUnknownUnoccupCells = [x for x in confidentUnknownCells if x not in confidentUnknownOccupCells]
    nConfidentUnknownOccup = len(confidentUnknownOccupCells)
    nConfidentUnknownUnoccup = len(confidentUnknownUnoccupCells)
    LOG('confident unknown: nUnoccup:%d nOccup:%d' % (nConfidentUnknownUnoccup, nConfidentUnknownOccup), file=outputFile)
    confidentIncorrectOccupCells = [x for x in confidentIncorrectCells if x < 1]
    confidentIncorrectUnoccupCells = [x for x in confidentIncorrectCells if x not in confidentIncorrectOccupCells]
    nConfidentIncorrectOccup = len(confidentIncorrectOccupCells)
    nConfidentIncorrectUnoccup = len(confidentIncorrectUnoccupCells)
    LOG('confident incorrect: nUnoccup:%d nOccup:%d' % (nConfidentIncorrectUnoccup, nConfidentIncorrectOccup), file=outputFile)
    
    nNotConfidentCorrect = len(notConfidentCorrectCells)
    nNotConfidentUnknown = len(notConfidentUnknownCells)
    nNotConfidentIncorrect = len(notConfidentIncorrectCells)
    nNotConfident = nNotConfidentCorrect + nNotConfidentUnknown + nNotConfidentIncorrect
    LOG('notConfident: nNotConfident:%d nCorrect:%d (%f) nIncorrect:%d (%f) nUnknown:%d (%f)' % (nNotConfident, nNotConfidentCorrect, nNotConfidentCorrect / nConfident, nNotConfidentIncorrect, nNotConfidentIncorrect / nNotConfident, nNotConfidentUnknown, nNotConfidentUnknown / nNotConfident), file=outputFile)
    notConfidentCorrectOccupCells = [x for x in notConfidentCorrectCells if x < 1]
    notConfidentCorrectUnoccupCells = [x for x in notConfidentCorrectCells if x not in notConfidentCorrectOccupCells]
    nNotConfidentCorrectOccup = len(notConfidentCorrectOccupCells)
    nNotConfidentCorrectUnoccup = len(notConfidentCorrectUnoccupCells)
    LOG('notConfident correct: nUnoccup:%d nOccup:%d' % (nNotConfidentCorrectUnoccup, nNotConfidentCorrectOccup), file=outputFile)
    notConfidentUnknownOccupCells = [x for x in notConfidentUnknownCells if x < 1]
    notConfidentUnknownUnoccupCells = [x for x in notConfidentUnknownCells if x not in notConfidentUnknownOccupCells]
    nNotConfidentUnknownOccup = len(notConfidentUnknownOccupCells)
    nNotConfidentUnknownUnoccup = len(notConfidentUnknownUnoccupCells)
    LOG('notConfident unknown: nUnoccup:%d nOccup:%d' % (nNotConfidentUnknownUnoccup, nNotConfidentUnknownOccup), file=outputFile)
    notConfidentIncorrectOccupCells = [x for x in notConfidentIncorrectCells if x < 1]
    notConfidentIncorrectUnoccupCells = [x for x in notConfidentIncorrectCells if x not in notConfidentIncorrectOccupCells]
    nNotConfidentIncorrectOccup = len(notConfidentIncorrectOccupCells)
    nNotConfidentIncorrectUnoccup = len(notConfidentIncorrectUnoccupCells)
    LOG('notConfident incorrect: nUnoccup:%d nOccup:%d' % (nNotConfidentIncorrectUnoccup, nNotConfidentIncorrectOccup), file=outputFile)