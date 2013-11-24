import math
from pdb import set_trace

STD_DEV_COST_COEFF = 200.0

BATTERY_LOSS_MOVE = 20.0

STD_DEV_MAX = 20



'''
gross - profit we est we'll get when we adopt the target
grossPerStep - how much the robot ests it will be getting for each step for this target
gross_subd - the profit we calc that this target will get when we redraw the local map for submission
gross_actual - gross_subd * profitRatio for the subd map (profit actually incd into global map / profit that the robot ests it will get when submitting)
'''

def analyseLocalMaps(robotIndex, incdMapsOnBoard, subdMapsOnRobot, moveList, mapScanList):
    '''
    Get relationship between estd profit, and actual profit.
    '''
    profitPerMoveOverStdDevs = []
    profitRatioOverStdDevs = []
    localMapAvgs = {}
    gtepSessionList = {}
    gtepSessionDict = {}
    incdMapsOnBoardFromMe = [x for x in incdMapsOnBoard if x['robotIndex'] == robotIndex]
    incdAndSubdMaps = zip(incdMapsOnBoardFromMe, subdMapsOnRobot)
    currentIteration = -1
    for mapTuple in incdAndSubdMaps:
        incdMap, subdMapProfit = mapTuple

        assert incdMap['robotIndex'] == subdMapProfit['robotIndex']
        assert incdMap['localMapIndex'] == subdMapProfit['localMapIndex']

        # Get further info about submitted local map: map scans integrated while exploring and moves made.
        movesThisMap = [x for x in moveList if x['iteration'] > currentIteration and x['iteration'] <= subdMapProfit['iteration']]
        #scansThisMap = [x for x in mapScanList if x['iteration'] > currentIteration and x['iteration'] <= subdMapProfit['iteration']]
        #estdGainFromScans = sum(x['estdGain'] for x in scansThisMap)
        nMoves = len(movesThisMap)

        # Take other interesting data and store in subdMapProfit.
        #subdMapProfit['estdGainFromScans'] = estdGainFromScans
        subdMapProfit['nMoves'] = nMoves
        profitRatio = (incdMap['profit'] / subdMapProfit['estdProfit']) if subdMapProfit['estdProfit'] else 0
        subdMapProfit['incdToSubdProfit'] = profitRatio
        subdMapProfit['incdToSubdNCells'] = (incdMap['nMapCells'] / subdMapProfit['estdGain']) if subdMapProfit['estdGain'] else 0
        subdMapProfit['incdToSubdAvgStdDev'] = (incdMap['avgMapCellStdDev'] / subdMapProfit['avgStdDev']) if subdMapProfit['avgStdDev'] else 0
        subdMapProfit['expProfit_actual'] = subdMapProfit['expProfit'] * profitRatio
        subdMapProfit['gtepProfit_actual'] = subdMapProfit['gtepProfit'] * profitRatio
        if nMoves:
            profitPerMoveOverStdDev = (incdMap['profit'] / nMoves, incdMap['avgMapCellStdDev'])
            profitPerMoveOverStdDevs.append(profitPerMoveOverStdDev)

            # TODO calculate actual profitRatio for each target in each local map. can really only do for exp tars
            # we already have for each target: gross, resources, expenditure
            # and for the subd local map we have the profit for each tar. therfore can use actualGross for incd map to
            # calc actual gross for each target. can update the targets with this info
            # (targets should have the id for each local map they have. then need a dict for local maps (with id as key))
            profitRatioOverStdDevs.append(profitRatioOverStdDev)

        for gtepSessionId, estdGtepProfit, nMapScans in subdMapProfit['gtepProfitList']:
            allocdGtepProfit = estdGtepProfit * profitRatio
            if not gtepSessionId in gtepSessionList:
                gtepSessionList[gtepSessionId] = {'estdProfit': 0.0, 'allocdProfit': 0.0, 'nLocalMaps': 0, 'nMapScans': 0}
            gtepSessionList[gtepSessionId]['estdProfit'] += estdGtepProfit
            gtepSessionList[gtepSessionId]['allocdProfit'] += allocdGtepProfit
            gtepSessionList[gtepSessionId]['nLocalMaps'] += 1
            gtepSessionList[gtepSessionId]['nMapScans'] += nMapScans

        currentIteration = subdMapProfit['iteration']

    # Get averages for all local maps.
    nMaps = len(subdMapsOnRobot)
    localMapAvgs['estdProfit'] = sum(x['estdProfit'] for x in subdMapsOnRobot) / nMaps
    localMapAvgs['incdToSubdProfit'] = sum(x['incdToSubdProfit'] for x in subdMapsOnRobot) / nMaps
    localMapAvgs['estdGain'] = sum(x['estdGain'] for x in subdMapsOnRobot) / nMaps
    localMapAvgs['incdToSubdNCells'] = sum(x['incdToSubdNCells'] for x in subdMapsOnRobot) / nMaps
    localMapAvgs['avgStdDev'] = sum(x['avgStdDev'] for x in subdMapsOnRobot) / nMaps
    localMapAvgs['nMoves'] = sum(x['nMoves'] for x in subdMapsOnRobot) / nMaps

    # Get averages for GOTO_EXPLORATION_PT sessions
    nSessions = len(gtepSessionList.keys())
    gtepSessionDict['avgEstdProfit'] = sum(x['estdProfit'] for x in gtepSessionList.values()) / nSessions if nSessions else 0
    gtepSessionDict['avgAllocdProfit'] = sum(x['allocdProfit'] for x in gtepSessionList.values()) / nSessions if nSessions else 0
    gtepSessionDict['avgNLocalMaps'] = sum(x['nLocalMaps'] for x in gtepSessionList.values()) / nSessions if nSessions else 0 # Remember, should be 1 apart from loop-closing
    gtepSessionDict['avgNMapScans'] = sum(x['nMapScans'] for x in gtepSessionList.values())

    # Setup dict to allow quick accessing of actual/estimated profit for each local map.
    localMapProfitPtrs = {}
    for localMap in subdMapsOnRobot:
        index = localMap['localMapIndex']
        incdToSubdProfit = localMap['incdToSubdProfit']
        assert index not in localMapProfitPtrs
        localMapProfitPtrs[index] = incdToSubdProfit

    return localMapAvgs, localMapProfitPtrs, gtepSessionDict, gtepSessionList, profitPerMoveOverStdDevs


def getAvgErrorPerMove(errorPerMoveTuples):
    '''Given tuples containing (nMoves, avgError), return the overall average error per move.'''
    nTotal = 0
    avgTotal = 0.0
    for (nMoves, avgError) in errorPerMoveTuples:
        if nMoves:
            avgTotal += avgError * nMoves
            nTotal += nMoves
    if nTotal:
        return (avgTotal / nTotal, nTotal)
    return (0.0, 0)

    
def getMoveBreakdown(moveList):
    total = len(moveList)
    fwdMoves = [x for x in moveList if x['direction'] == 0]
    return len(fwdMoves), total - len(fwdMoves)

    
def updateExpTargetsWrtEnvir(expTargetsList, profitWrtEnvir):
    """
    When printing what exploration targets look like relative to the environment, i.e. the state of the local map and the sup area,
    it will also be useful to determine how the envir typically changes with each target.
    """
    for iteration in profitWrtEnvir:
        iterationDict = profitWrtEnvir[iteration]
        if iterationDict['exp']:
            matchingExpTargets = [x for x in expTargetsList if x['iteration'] == iteration]
            if not matchingExpTargets:
                print('WARNING: should have a matching exp dict for iteration %d' % iteration)
                iterationDict['exp']['mapGain_subd'] = 0
            else:
                iterationDict['exp']['mapGain_subd'] = matchingExpTargets[0]['mapGain_subd']


def analyseResultsDictList(resultsDictList):
    temp_nExpTars = 0
    temp_nExpTarsWithCoalSet = 0
    
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'subdMapList_%d' % index not in resultsDict:
                index += 1
                continue

            avgErrorPerMove, nMoves = getAvgErrorPerMove(resultsDict['globalMapDetails']['errorPerMoveList'])
            resultsDict['globalMapDetails']['movesAvgError'] = avgErrorPerMove
            resultsDict['globalMapDetails']['movesN'] = nMoves

            nRotations, nFwdMoves = getMoveBreakdown(resultsDict['globalMapDetails']['moveList'])
            resultsDict['globalMapDetails']['movesDirRotations'] = nRotations
            resultsDict['globalMapDetails']['movesDirFwd'] = nFwdMoves
            resultsDict['globalMapDetails']['movesDirRatioRotations'] = nRotations / float(nFwdMoves)

            subdMapsList = resultsDict['subdMapList_%d' % index]
            tempSubdMapDict = {}  # Map localMapIndex to subdMap dict object
            for subdMap in subdMapsList:
                if subdMap['estdGross'] == 0.0:
                    subdMap['estdGross'] = 1.0
                subdMap['profitRatio'] = subdMap['incdProfit'] / subdMap['estdGross']
                subdMap['profitRatioList'] = []
                assert subdMap['localMapIndex'] not in tempSubdMapDict
                tempSubdMapDict[subdMap['localMapIndex']] = subdMap
            
            coalitionInfoList = resultsDict['coalitionInfo_%d' % index]

            gtepTargetsList = resultsDict['gtepTargetList_%d' % index]
            tempGtepTargetDict = {}
            for gtepTarget in gtepTargetsList:
                if gtepTarget['localMapIndex'] not in tempSubdMapDict:
                    set_trace()
                    print("ERROR")
                assert gtepTarget['localMapIndex'] in tempSubdMapDict
                gtepTarget['nScans_actual'] = 0
                gtepTarget['gross_subd'] = 0
                gtepTarget['gross_actual'] = 0
                gtepTarget['nLocalMaps'] = 0

                for subdMap in subdMapsList:
                    targetProfits = [x for x in subdMap['targetProfits'] if x['id'] == gtepTarget['targetIndex']]
                    assert len(targetProfits) in (0, 1)
                    if targetProfits:
                        gtepTarget['nScans_actual'] += targetProfits[0]['nScans']
                        gtepTarget['gross_subd'] += targetProfits[0]['profit']
                        gtepTarget['gross_actual'] += targetProfits[0]['profit'] * subdMap['profitRatio']
                        gtepTarget['nLocalMaps'] += 1
                grossRatio = gtepTarget['gross_actual'] / gtepTarget['gross_subd'] if gtepTarget['gross_subd'] else 0.0
                gtepTarget['expenditure_actual'] = gtepTarget['expenditure'] * grossRatio

                # TODO: N.B. Make sure these match values in c code
                gtepTarget['resources_actual'] = gtepTarget['nScans_actual_redundant'] * BATTERY_LOSS_MOVE + gtepTarget['stdDevInc_actual'] * STD_DEV_COST_COEFF
                gtepTarget['ratio_actual'] = (gtepTarget['gross_actual'] - gtepTarget['expenditure_actual']) / gtepTarget['resources_actual'] if gtepTarget['resources_actual'] else 0

                # Arbitrary - add it to the first local map that this target had any scans in - mostly only one anyway.
                tempSubdMapDict[gtepTarget['localMapIndex']]['profitRatioList'].append(gtepTarget['ratio_actual'])

                # List for allocated profit. There may be multiple gtep targets with the same 
                # potentialGtepSessionId, i.e. some will be adopted but the session not started. This will
                # be ok tho, as the last gtep target in the list will overwrite the previous ones.
                gtepTarget['allocatedProfitList'] = []
                tempGtepTargetDict[gtepTarget['potentialGtepSessionId']] = gtepTarget
            
            expTargetsList = resultsDict['expTargetList_%d' % index]
            for expTarget in expTargetsList:
                temp_nExpTars += 1
                
                assert expTarget['localMapIndex'] in tempSubdMapDict
                expTarget['nScans_actual'] = 0
                expTarget['gross_subd'] = 0
                expTarget['mapGain_subd'] = 0
                expTarget['gross_actual'] = 0
                expTarget['nLocalMaps'] = 0
                
                for subdMap in subdMapsList:
                    targetProfits = [x for x in subdMap['targetProfits'] if x['id'] == expTarget['targetIndex']]
                    assert len(targetProfits) in (0, 1)
                    if targetProfits:
                        expTarget['nScans_actual'] += targetProfits[0]['nScans']
                        expTarget['gross_subd'] += targetProfits[0]['profit']
                        expTarget['mapGain_subd'] += targetProfits[0]['estdGain']
                        expTarget['gross_actual'] += targetProfits[0]['profit'] * subdMap['profitRatio']
                        expTarget['nLocalMaps'] += 1
                grossRatio = expTarget['gross_actual'] / expTarget['gross_subd'] if expTarget['gross_subd'] else 0.0
                expTarget['expenditure_actual'] = expTarget['expenditure'] * grossRatio

                expTarget['grossPerStep'] = expTarget['gross'] / expTarget['nSteps']
                expTarget['grossPerStep_actual'] = expTarget['gross_actual'] / expTarget['nSteps']

                # TODO: N.B. Make sure these match values in c code
                expTarget['resources_actual'] = expTarget['nScans_actual_redundant'] * BATTERY_LOSS_MOVE + expTarget['stdDevInc_actual'] * STD_DEV_COST_COEFF
                expTarget['ratio_actual'] = (expTarget['gross_actual'] - expTarget['expenditure_actual']) / expTarget['resources_actual'] if expTarget['resources_actual'] else 0
                
                # Arbitrary - add it to the first local map that this target had any scans in - mostly only one anyway.
                tempSubdMapDict[expTarget['localMapIndex']]['profitRatioList'].append(expTarget['ratio_actual'])

                if expTarget['gtepSession'] != -1:
                    profitToGtep_actual = expTarget['profitToGtep'] * grossRatio
                    tempGtepTargetDict[expTarget['gtepSession']]['allocatedProfitList'].append(profitToGtep_actual)

                if expTarget['profitToCoal'] != 0:
                    temp_nExpTarsWithCoalSet += 1

                    assert expTarget['iteration'] in coalitionInfoList
                    coalition = coalitionInfoList[expTarget['iteration']]['explorationCoalition']
                    if coalition not in resultsDict['globalMapDetails']['coalInfo']:
                        resultsDict['globalMapDetails']['coalInfo'][coalition] = {'coalAllocs': [], 'subdMapProfits': []}
                    profitToCoal_actual = expTarget['profitToCoal'] * grossRatio
                    resultsDict['globalMapDetails']['coalInfo'][coalition]['coalAllocs'].append(profitToCoal_actual)
                
            # Count up profit from allocatedProfitList allocd above
            for gtepTarget in gtepTargetsList:
                gtepTarget['allocd_income'] = sum(gtepTarget['allocatedProfitList'])
                gtepTarget['allocd_nPayments'] = len(gtepTarget['allocatedProfitList'])
                gtepTarget['allocd_income_gross_ratio'] = gtepTarget['allocd_income'] / gtepTarget['gross'] if gtepTarget['gross'] else 0
                #gtepTarget['allocd_ratio_actual_after'] = (gtepTarget['gross_actual'] + gtepTarget['income'] - gtepTarget['expenditure_actual']) / gtepTarget['resources_actual'] if gtepTarget['resources_actual'] else 0

            # Calc average profit ratio for each target in each local map
            for subdMap in subdMapsList:
                profitRatioList = subdMap['profitRatioList']
                subdMap['avgProfitRatio'] = sum(profitRatioList) / len(profitRatioList) if profitRatioList else 0.0

            # Process close loop data
            closeLoopSessionList = resultsDict['closeLoopSessions_%d' % index]
            for session in closeLoopSessionList:
                target = session['target']
                
                session['nScansTotalWhenAdopting'] = target['additionalInfo']['nScansTotal']
                session['nScansInitial'] = target['additionalInfo']['nScansInitial']
                session['nScansLoopClosingWhenAdopting'] = target['additionalInfo']['nScansCloseLoop']
                session['nScansAdjustedWhenAdopting'] = target['additionalInfo']['nScansAdjusted']
                session['nScansAdjustedInitialWhenAdopting'] = max(0, session['nScansAdjustedWhenAdopting'] - session['nScansLoopClosingWhenAdopting'])
                    
                maxScanWhenAdopting = max(target['scanBehaviours'].keys())
                session['lastStdDevWhenAdopting'] = target['scanBehaviours'][maxScanWhenAdopting]['origStdDev']

                adjustedScansWhenSubmitting = [x for x in session['scanAdjustments'] if session['scanAdjustments'][x]['event'] == 'adjust']
                session['nScansAdjustedWhenSubmitting'] = len(adjustedScansWhenSubmitting)

                session['nScansTotalWhenSubmitting'] = len(session['perScanProfits'])
                session['nScansLoopClosingWhenSubmitting'] = len(session['perScanProfits']) - target['additionalInfo']['nScansInitial']
                session['nScansAdjustedWhenSubmitting'] = max(0, session['nScansAdjustedWhenSubmitting'] - session['nScansLoopClosingWhenSubmitting'])

                maxScanWhenSubmitting = max(session['scanAdjustments'].keys())
                session['lastStdDevWhenSubmitting'] = session['scanAdjustments'][maxScanWhenSubmitting]['origStdDev']
                
                # From submitted scans, calc gross from new scans and from adjustments
                #l1 = [x for x in session['scanAdjustments'].keys() if x not in session['perScanProfits'].keys()]
                #if l1:
                #    set_trace()
                for scanIndex in session['scanAdjustments']:
                    scanAdj = session['scanAdjustments'][scanIndex]
                    
                    # A scan may not appear in 'perScanProfits', if when doing a loop-close, one map had no map data in it (i.e. all scans had too high std dev)
                    if scanIndex in session['perScanProfits']:
                        scanAdj['profit'] = session['perScanProfits'][scanIndex]['profit']
                        scanAdj['gain'] = session['perScanProfits'][scanIndex]['gain']
                    else:
                        scanAdj['profit'] = 0
                        scanAdj['gain'] = 0
                    scanAdj['wasInitialScan'] = (scanIndex < session['nScansInitial'])

                initialScans = [x for x in session['scanAdjustments'].values() if x['wasInitialScan']]
                newScans = [x for x in session['scanAdjustments'].values() if not x['wasInitialScan']]
                
                
                adjustmentGross = 0
                for scanAdj in initialScans:
                    if scanAdj['event'] == 'adjust':
                        if scanAdj['adjStdDev'] >= scanAdj['origStdDev']:
                            print('WEIRD: adjusted std dev should have been smaller than original std dev')
                        # Calc gross as in calcStdDevReductionProfit in code
                        red = scanAdj['origStdDev'] - scanAdj['adjStdDev']
                        adjustmentGross += scanAdj['gain'] * (red / STD_DEV_MAX)
                session['grossAdjsWhenSubmitting'] = adjustmentGross
                
                newScanGross = 0
                for scanAdj in newScans:
                    newScanGross += scanAdj['profit']
                session['grossNewScansWhenSubmitting'] = newScanGross

                session['grossAdjsWhenAdopting'] = target['additionalInfo']['grossAdjs']
                session['grossNewScansWhenAdopting'] = target['additionalInfo']['grossNewScans']
                
                # Calc errors before/after adjustment in adjusted scans
                def _pointDist(pt1, pt2):
                    assert len(pt1) == 2
                    assert len(pt2) == 2
                    xDist = abs(pt1[0] - pt2[0])
                    yDist = abs(pt1[1] - pt2[1])
                    dist = math.sqrt(xDist * xDist + yDist * yDist)
                    return dist

                def __diffLen(pt):
                    assert len(pt) == 2
                    dist = math.sqrt(pt[0] * pt[0] + pt[1] * pt[1])
                    return dist

                for scanIndex in session['scanAdjustments']:
                    scanAdj = session['scanAdjustments'][scanIndex]

                    scanAdj['offsetOrig'] = __diffLen(scanAdj['origDiff'])
                    if 'adjDiff' in scanAdj:
                        scanAdj['offsetAdj'] = __diffLen(scanAdj['adjDiff'])
                
                reversedKeys = list(reversed(sorted(session['scanAdjustments'].keys())))
                scanAdj = session['scanAdjustments'][reversedKeys[0]]
                session['scanAdjsAvgIncOffset'] = ''
                session['scanAdjsNormdIncOffset'] = ''
                session['scanAdjsAvgIncStdDev'] = ''
                if 'offsetAdj' in scanAdj:
                    currentOffset = scanAdj['offsetAdj']
                    currentStdDev = scanAdj['adjStdDev']
                    offsetIncs = []
                    stdDevIncs = []
                    for scanIndex in reversedKeys[1:]:
                        scanAdj = session['scanAdjustments'][scanIndex]
                        if 'offsetAdj' in scanAdj:
                            offsetInc = scanAdj['offsetAdj'] - currentOffset
                            offsetIncs.append(offsetInc)
                            currentOffset = scanAdj['offsetAdj']
                            stdDevInc = scanAdj['adjStdDev'] - currentStdDev
                            stdDevIncs.append(stdDevInc)
                            currentStdDev = scanAdj['adjStdDev']
                    if offsetIncs:
                        #set_trace()
                        session['scanAdjsAvgIncOffset'] = sum(offsetIncs) / len(offsetIncs)
                        session['scanAdjsNormdIncOffset'] = session['scanAdjsAvgIncOffset'] * len(offsetIncs)
                        session['scanAdjsAvgIncStdDev'] = sum(stdDevIncs) / len(stdDevIncs)
            
            profitWrtEnvir = resultsDict['profitWrtEnvir_%d' % index]
            updateExpTargetsWrtEnvir(expTargetsList, profitWrtEnvir)
            index += 1
        
        # Iterate again, once profit allocd to coalitions has been counted for all robots
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'subdMapList_%d' % index not in resultsDict:
                index += 1
                continue

            # For exploration (which is the only behaviour for which profit is directly attribd for map gain ), we
            # have already subtracted (in c code) the profit to attrib to the coal (this is in the step below). It
            # would/may be more correct to attrib profit from all profits to coal though. This will not mess up our
            # graphs either, as this profit is not counted anyway.
            coalitionInfoList = resultsDict['coalitionInfo_%d' % index]
            subdMapsList = resultsDict['subdMapList_%d' % index]
            for subdMap in subdMapsList:
                if subdMap['iteration'] in coalitionInfoList:
                    coalitionId = coalitionInfoList[subdMap['iteration']]['explorationCoalition']
                    if coalitionId != -1:

                        # It may occur that a coal gets no exp targets or subd maps, so it won't be added to this dict,
                        # so add a dict with empty lists in this case
                        if coalitionId not in resultsDict['globalMapDetails']['coalInfo']:
                            print('WARNING: looks like no profit attribd to coal id=%d' % coalitionId)
                            resultsDict['globalMapDetails']['coalInfo'][coalitionId] = {'coalAllocs': [], 'subdMapProfits': []}

                        coalitionDict = resultsDict['globalMapDetails']['coalInfo'][coalitionId]
                        profitToCoal = subdMap['incdProfit'] * 0.01  # moveCost=20, idleCost=0.2
                        subdMap['profitToCoal'] = profitToCoal
                        coalitionDict['subdMapProfits'].append(profitToCoal)

            supTargetsList = resultsDict['supTargetList_%d' % index]
            for supTarget in supTargetsList:
                if supTarget['coalId'] not in resultsDict['globalMapDetails']['coalInfo']:
                    supTarget['nAllocs'] = 0
                    supTarget['profitAllocd'] = 0
                    supTarget['nSubdMaps'] = 0
                    supTarget['profitSubdMaps'] = 0
                else:
                    coalDict = resultsDict['globalMapDetails']['coalInfo'][supTarget['coalId']]
                    allocList = coalDict['coalAllocs']
                    subdMapList = coalDict['subdMapProfits']
                    supTarget['nAllocs'] = len(allocList)
                    supTarget['profitAllocd'] = sum(allocList)
                    supTarget['nSubdMaps'] = len(subdMapList)
                    supTarget['profitSubdMaps'] = sum(subdMapList)
            index += 1

    print('temp_nExpTars=%d temp_nExpTarsWithCoalSet=%d\n' % (temp_nExpTars, temp_nExpTarsWithCoalSet))