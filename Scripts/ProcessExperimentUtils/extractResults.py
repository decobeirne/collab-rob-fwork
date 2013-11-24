from pdb import set_trace
import re
import math

STD_DEV_PIXEL_GAP = 20

STD_DEV_MAX = 20


SUB_PYTHONIFY = re.compile('(?P<name>\w+)=(?P<val>\S+)').sub
def pythonify(logText):
    '''Convert XML name-value pairs to the syntax for a python dictionary.'''
    pythonText = '{' + SUB_PYTHONIFY('"\g<name>":\g<val>,', logText) + '}'
    return pythonText


FIND_LOCAL_MAP_PROFIT = re.compile(r'<LocalMapProfit>(.+?)</LocalMapProfit>').finditer
def getIncdMapsOnBoard(boardFile):
    '''Extract <LocalMapProfit> tags'''
    iter = FIND_LOCAL_MAP_PROFIT(open(boardFile).read())
    incdMaps = []
    for m in iter:
        logText = m.group(1)
        pythonText = pythonify(logText)
        mapDict = eval(pythonText)
        incdMaps.append(mapDict)
    return incdMaps

FINDITER_SUP_STATS_ON_BOARD = re.compile(r'<GlobalMapStats>index=([0-9]+).+?<SupAreasPixelsMapped>(.+?)</SupAreasPixelsMapped>.+?<SupAreasExpCellsMapped>(.+?)</SupAreasExpCellsMapped>.+?</GlobalMapStats>', re.DOTALL).finditer
def getSupGridsStatsOnBoard(boardFile):
    text = open(boardFile).read()
    iter = FINDITER_SUP_STATS_ON_BOARD(text)
    supGridStatusDict = {}
    for m in iter:
        experimentIteration = int(m.group(1))
        if experimentIteration in supGridStatusDict:
            print('WARNING: duplicate iteration found when searching for global map stats - %d (may just be from final update)' % experimentIteration)
            continue
        supGridStatusDict[experimentIteration] = {}
        iterDict = supGridStatusDict[experimentIteration]

        pixelsInfoList = m.group(2).split(';')
        for pixelsInfo in pixelsInfoList:
            if not pixelsInfo.strip():
                continue
            pixelsDict = eval(pythonify(pixelsInfo))
            id = pixelsDict['id']
            if id not in iterDict:
                iterDict[id] = {}
            iterDict[id]['nPixelsMapped'] = pixelsDict['nPixelsMapped']

        cellsInfoList = m.group(3).split(';')
        for cellsInfo in cellsInfoList:
            if not cellsInfo.strip():
                continue
            cellsDict = eval(pythonify(cellsInfo))
            id = cellsDict['id']
            if id not in iterDict:
                iterDict[id] = {}
            iterDict[id]['nExpCellsMapped'] = cellsDict['nExpCellsMapped']
            iterDict[id]['nLocalMapsExhausted'] = cellsDict['nLocalMapsExhausted']
    return supGridStatusDict


FINDITER_STATUS = re.compile('<RobotStatus>iteration=(\S+)').finditer  # Could obviously just use a str func here, but may want to change pattern later
SEARCH_COAL_INFO = re.compile('<CoalitionStatus>(.+?)</CoalitionStatus>').search
def getCoalitionInfo(fileText):
    iter = FINDITER_STATUS(fileText)
    coalitionInfo = {}
    for m in iter:
        iteration = int(m.group(1))
        start = m.start()
        coalitionInfo[iteration] = {'start': start}

    iterations = sorted(coalitionInfo.keys())
    for iteration in iterations:
        nextIteration = iteration + 1
        if nextIteration in iterations:
            coalitionInfo[iteration]['end'] = coalitionInfo[nextIteration]['start']
        else:
            coalitionInfo[iteration]['end'] = None

    for iteration in iterations:
        info = coalitionInfo[iteration]
        if info['end']:
            coalMatch = SEARCH_COAL_INFO(fileText, info['start'], info['end'])
        else:
            coalMatch = SEARCH_COAL_INFO(fileText, info['start'])
        assert coalMatch    
        coalDict = eval(pythonify(coalMatch.group(1)))
        info['explorationCoalition'] = coalDict['explorationCoalitionId']
        info['supervisionCoalitions'] = coalDict['supervisionCoalitionIds']
    return coalitionInfo


SEARCH_GLOBAL_MAP = re.compile('<GlobalMap>(.+?)</GlobalMap>').search
def getGlobalMapDetails(boardFile):
    '''Extract <GlobalMap> tags.'''
    fileText = open(boardFile).read()
    m = SEARCH_GLOBAL_MAP(fileText)
    
    mapText = m.group(1)
    mapText = pythonify(mapText)
    mapDict = eval(mapText)

    # Std dev is encoded in the pixel value as pixelOffsetFrom127 = gap + stdDevMax - stdDev,
    # i.e. a value of (gap + stdDevMax) would be a stdDev of 0. gap=30, stdDevMax=30 are currently used
    
    stdDevTotal = 0.0
    nCellsTotal = 0.0
    #stdDevConst = 60  # gap + stdDevMax, where gap=30 and stdDevMax=30
    stdDevConst = STD_DEV_PIXEL_GAP + STD_DEV_MAX

    for (cellValue, nCells) in mapDict['cells']:
        diff = abs(cellValue - 127)
        stdDev = stdDevConst - diff
        stdDevVal = nCells * stdDev
        stdDevTotal += stdDevVal
        nCellsTotal += nCells

    assert nCellsTotal == mapDict['nMapped']
    stdDevAvg = stdDevTotal / nCellsTotal

    globalMapDict = {}
    globalMapDict['nMapped'] = mapDict['nMapped']
    globalMapDict['nCells'] = mapDict['nCells']
    globalMapDict['nOccupied'] = mapDict['nOccupied']
    globalMapDict['avgStdDev'] = stdDevAvg
    return globalMapDict


FINDITER_ADOPT_GTEP = re.compile(r'<AdoptBehaviour>behaviour="(GOTO_EXP_PT|GOTO_EXP_PT_COLLAB)"(.+?)</AdoptBehaviour>').finditer
FINDITER_ABORT_GTEP = re.compile(r'<AbortGTEPSession>(.+?)</AbortGTEPSession>').finditer
FINDITER_START_GTEP = re.compile(r'<StartGTEPSession>(.+?)</StartGTEPSession>').finditer
FINDITER_GTEP_PROFIT = re.compile(r'<GTEPProfit>(.+?)</GTEPProfit>').finditer
SEARCH_POT_GTEP_SESSION = re.compile('potentialGtepSessionId=([0-9]+)').search
SEARCH_GTEP_SESSION = re.compile('gtepSessionId=([0-9]+)').search


def sanityCheckFile(fileText):
    '''Find GOTO_EXP_PT sessions and compare sessions adopted, started and aborted.'''
    print('Sanity check')

    print('Adoption of GOTO_EXP_PT')
    adoptedSessions = []
    abortedSessions = []
    startedSessions = []
    submittedScanGtepSessions = []
    for m in FINDITER_ADOPT_GTEP(fileText):
        adoptGtepText = m.group(2)
        session = SEARCH_POT_GTEP_SESSION(adoptGtepText)
        adoptedSessions.append(session.group(1))
    for m in FINDITER_ABORT_GTEP(fileText):
        abortGtepText = m.group(1)
        session = SEARCH_POT_GTEP_SESSION(abortGtepText)
        abortedSessions.append(session.group(1))
    for m in FINDITER_START_GTEP(fileText):
        startGtepText = m.group(1)
        session = SEARCH_GTEP_SESSION(startGtepText)
        startedSessions.append(session.group(1))
    for m in FINDITER_GTEP_PROFIT(fileText):
        gtepText = m.group(1)
        if gtepText != '[]':
            gtepProfitList = eval(gtepText)
            for gtepTuple in gtepProfitList:
                session = gtepTuple[0]  # Tuple should contain (sessionId, estdProfit, nScans)
                submittedScanGtepSessions.append(session)
    print('Adopted sessions')
    print(adoptedSessions)
    originalAdoptedSessions = adoptedSessions[:]
    print('Aborted sessions')
    print(abortedSessions)
    print('Started sessions')
    print(startedSessions)
    print('Sessions accredited in the robot\'s gtepProfitList when submitting local maps')
    print(submittedScanGtepSessions)
    for x in abortedSessions:
        index = adoptedSessions.index(x)
        assert index != -1
        adoptedSessions.pop(index)
    print('Adopted sessions not aborted')
    print(adoptedSessions)
    tempList = list(set(startedSessions))
    assert sorted(tempList) == sorted(startedSessions)

    for x in startedSessions:
        index = adoptedSessions.index(x)
        assert index != -1
        adoptedSessions.pop(index)

    #if len(adoptedSessions) == 1 and (int(adoptedSessions[0]) == max([int(x) for x in originalAdoptedSessions])):  # Catch when exp ends during last session
    #    abortedSessions.append(adoptedSessions.pop(0))
    #    print('Switchng session %s to adopted' % abortedSessions[-1])
    #assert not adoptedSessions
    
    # Assume this is working for now
    while adoptedSessions:
        abortedSessions.append(adoptedSessions.pop(0))
        print('Switchng session %s to adopted' % abortedSessions[-1])
        
    print('')

    return len(abortedSessions), len(startedSessions)


FINDITER_MOVE = re.compile(r'<Move>(.+?)</Move>', re.DOTALL).finditer


def getRobotMoves(fileText):
    '''Extract <Move> tags'''
    stdDevBefore = 0.0
    errorsPerMove = []
    loc1 = fileText.rfind('iteration=')
    loc2 = fileText.find(' ', loc1)
    lastIteration = int(fileText[loc1 + 10:loc2])

    iter = FINDITER_MOVE(fileText)
    moves = []
    for m in iter:
        moveText = m.group(1)
        pythonText = pythonify(moveText)
        moveDict = eval(pythonText)
        index1 = fileText.find('stdDev=', m.end())
        index2 = fileText.find(' ', index1)
        stdDevAfter = float(fileText[index1 + 7:index2])
        errorsPerMove.append(stdDevAfter - stdDevBefore)
        stdDevBefore = stdDevAfter
        moves.append(moveDict)
    errorPerMoveResult = (0, 0)
    if errorsPerMove:
        errorPerMove = sum(errorsPerMove) / len(errorsPerMove)
        errorPerMoveResult = (len(errorsPerMove), errorPerMove)
    return moves, lastIteration, errorPerMoveResult


FINDITER_OFFSET = re.compile(r'<RobotStatus>.+?behaviour="(.+?)".+?actLocOff=(.+?)</RobotStatus>', re.DOTALL).finditer

def getOffsets(fileText):
    iter = FINDITER_OFFSET(fileText)
    offsets = []
    for m in iter:
        behText = m.group(1)
        if behText == 'STUCK':
            return offsets

        offsetText = m.group(2)
        offsetTuple = eval(offsetText)
        dist = math.sqrt(offsetTuple[0]*offsetTuple[0] + offsetTuple[1]*offsetTuple[1])
        offsets.append(dist)
    return offsets


FINDITER_INT_SCAN = re.compile('<IntegrateScan>(.+?)</IntegrateScan>').finditer


def getMapScans(fileText):
    '''Extract <IntegrateScan> tags'''
    scanTextList = FINDITER_INT_SCAN(fileText)
    scanList = []
    for scan in scanTextList:
        tempDict = eval(pythonify(scan.group(1)))
        scanList.append(tempDict)
    return scanList


FINDITER_STD_DEV = re.compile('<MoveStdDev>(.+?)</MoveStdDev>').finditer


def getStdDevIncs(fileText):
    '''Extract <MoveStdDev> tags'''
    stdDevIter = FINDITER_STD_DEV(fileText)
    stdDevList = []
    for stdDevMatch in stdDevIter:
        tempDict = eval(pythonify(stdDevMatch.group(1)))
        stdDevList.append(tempDict)
    return stdDevList


FINDITER_SUB_MAP = re.compile('<SubmitLocalMap>(.+?)</SubmitLocalMap>').finditer
SEARCH_BEH_PROFIT = re.compile('<PerBehaviour>(.+?)</PerBehaviour>').search
FINDITER_TAR_PROFIT = re.compile('<PerTarget>(.+?)</PerTarget>').finditer
FINDALL_REASON = re.compile('reason="(.+)"').findall
SEARCH_ITERATOR = re.compile('iteration=([0-9]+)').search

FINDITER_CLOSE_LOOP_SESSION = re.compile('<CloseLoopPerformed />').finditer
FINDITER_PER_SCAN_PROFIT = re.compile('<PerScan>(.+?)</PerScan>').finditer
FINDITER_PER_SCAN_ADJUSTMENT = re.compile('<AdjustScan>(.+?)</AdjustScan>').finditer


def getSubdMapsOnRobot(fileText):
    '''Extract <SubmitLocalMap> tags'''
    
    def __getSubdMapDict(subdMapMatch):
        subdMapText = subdMapMatch.group(1)
        subdMapText = pythonify(subdMapText)
        subdMapDict = eval(subdMapText)

        subdMapIndex = subdMapMatch.start()
        statusIndex = fileText.rfind('<RobotStatus>', 0, subdMapIndex)

        iteration = SEARCH_ITERATOR(fileText, statusIndex, subdMapIndex).group(1)
        iteration = int(iteration)
        reason = FINDALL_REASON(fileText, statusIndex, subdMapIndex)[-1]
        subdMapDict['iteration'] = iteration
        subdMapDict['reason'] = reason
        return (subdMapDict, subdMapIndex, statusIndex)

    subdMaps = []
    subdMapIter = FINDITER_SUB_MAP(fileText)
    for subdMapMatch in subdMapIter:
        subdMapDict, subdMapIndex, statusIndex = __getSubdMapDict(subdMapMatch)

        behaviourProfitMatch = SEARCH_BEH_PROFIT(fileText, statusIndex, subdMapIndex)
        if behaviourProfitMatch:
            behaviourProfitText = behaviourProfitMatch.group(1)
            behaviourProfitText = pythonify(behaviourProfitText)
            behaviourProfitDict = eval(behaviourProfitText)
            subdMapDict['behaviourProfits'] = behaviourProfitDict

        subdMapDict['targetProfits'] = []
        targetIter = FINDITER_TAR_PROFIT(fileText, statusIndex, subdMapIndex)
        for targetMatch in targetIter:
            targetProfitText = targetMatch.group(1)
            targetProfitText = pythonify(targetProfitText)
            targetProfitDict = eval(targetProfitText)
            subdMapDict['targetProfits'].append(targetProfitDict)

        subdMaps.append(subdMapDict)
    
    closeLoopSessions = []
    closeLoopSessionIter = FINDITER_CLOSE_LOOP_SESSION(fileText)
    for closeLoopSessionMatch in closeLoopSessionIter:
        closeLoopDict = {}
        
        tagIndex = closeLoopSessionMatch.start()
        startIndex = fileText.rfind('<RobotStatus>', 0, tagIndex)
        endIndex = fileText.find('<RobotStatus>', startIndex + 13)
        
        iteration = SEARCH_ITERATOR(fileText, startIndex, endIndex).group(1)
        iteration = int(iteration)
        reason = ""
        reasons = FINDALL_REASON(fileText, startIndex, endIndex)
        if reasons:
            reason = reasons[-1]
        closeLoopDict['iteration'] = iteration
        closeLoopDict['reason'] = reason

        closeLoopDict['perScanProfits'] = {}
        perScanIter = FINDITER_PER_SCAN_PROFIT(fileText, startIndex, endIndex)
        for perScanMatch in perScanIter:
            scanText = pythonify(perScanMatch.group(1))
            scanDict = eval(scanText)
            scan = scanDict['scan']
            scanDict.pop('scan')
            closeLoopDict['perScanProfits'][scan] = scanDict

        closeLoopDict['scanAdjustments'] = {}
        perScanIter = FINDITER_PER_SCAN_ADJUSTMENT(fileText, startIndex, endIndex)
        for perScanMatch in perScanIter:
            text = perScanMatch.group(1)
            tempDict = eval(pythonify(text))
            scan = tempDict['scan']
            closeLoopDict['scanAdjustments'][scan] = tempDict

        closeLoopSessions.append(closeLoopDict)

    return (subdMaps, closeLoopSessions)


def matchSubdAndIncdMaps(robotIndex, incdMapsOnBoard, subdMapsOnRobot):
    '''Match map submitted by robot to map incd on board.'''
    incdMapsOnBoardFromMe = [x for x in incdMapsOnBoard if x['robotIndex'] == robotIndex]
    for subdMap in subdMapsOnRobot:
        matchingMaps = [x for x in incdMapsOnBoardFromMe if x['localMapIndex'] == subdMap['localMapIndex']]
        if not matchingMaps:
            print('ERROR couldn\'t find matching incd map for %d' % subdMap['localMapIndex'])
        incdMap = matchingMaps[0]
        assert incdMap['robotIndex'] == subdMap['robotIndex']
        assert incdMap['localMapIndex'] == subdMap['localMapIndex']
        subdMap['incdProfit'] = incdMap['profit']
        subdMap['incdAvgStdDev'] = incdMap['avgStdDev']
        subdMap['incdMapGain'] = incdMap['mapGain']


def matchCloseLoopSessions(closeLoopSessions, closeLoopTargetList):

    # Match close loop sessions (when close loop was performed) with targets (when close
    # loop behaviour was adopted)
    sessionTuples = [(x['iteration'], x) for x in closeLoopSessions]
    sessionTuples.sort(key=lambda x: x[0])
    targetTuples = [(x['iteration'], x) for x in closeLoopTargetList]
    targetTuples.sort(key=lambda x: x[0])
    
    # Start at first session, get corresponding target (last one where iteration is less
    # that session iteration), and remove all targets up to and including that - there may
    # be more that one target when we don't adopt close loop
    for sessionTuple in sessionTuples:
        sessionIteration, session = sessionTuple
        
        earlierTargets = [x for x in targetTuples if x[0] <= sessionIteration]
        targetIteration, target = earlierTargets[-1]
        for targetTuple in earlierTargets:
            targetTuples.remove(targetTuple)
        
        print('Match close loop session from i=%d with target from i=%d' % (sessionIteration, targetIteration))
        session['target'] = target
        session['iterationAdopting'] = targetIteration
        session['iterationSubmitting'] = sessionIteration
    return


FINDITER_HIGH_LEVEL_BEH = re.compile('<HighLevelBehavior />', re.DOTALL).finditer
SEARCH_LOC_MAP_INDEX = re.compile('localMapIndex=([0-9]+)').search
SEARCH_ACT_LOC_OFFSET = re.compile('actLocOff=(\(.+?\))').search
SEARCH_ADOPT_BEH = re.compile('<AdoptBehaviour>(.+?)</AdoptBehaviour>').search
SEARCH_EXP_PROFIT =  re.compile('<MaxProfit>behaviour="EXPLORATION" (.+?)</MaxProfit>').search
SEARCH_SUP_PROFIT =  re.compile('<MaxProfit>behaviour="SUPERVISION" (.+?)</MaxProfit>').search
#SEARCH_SUP_COAL = re.compile('<ReadCoalitions>.+?supCoalitions=\(\((.+?)\),\)</ReadCoalitions>').search
SEARCH_SUP_COAL = re.compile('<ReadCoalitions>.+?supCoalitions=\((.+?)\)</ReadCoalitions>').search
SEARCH_GTEP_PROFIT = re.compile('<MaxProfit>behaviour="GOTO_EXP_PT" (.+?)</MaxProfit>').search
SEARCH_GTEP_PROFIT_COLLAB = re.compile('<MaxProfit>behaviour="GOTO_EXP_PT_COLLAB" (.+?)</MaxProfit>').search
SEARCH_CLOSE_LOOP_PROFIT = re.compile('<MaxProfit>behaviour="CLOSE_LOOP" (.+?)</MaxProfit>').search
FINDITER_CLOSE_LOOP_SCANS = re.compile('<CloseLoopScan>(.+?)</CloseLoopScan>', re.DOTALL).finditer
SEARCH_CLOSE_LOOP_PROFIT_INFO = re.compile('<CloseLoopProfit>(.+?)</CloseLoopProfit>').search


FINDITER_ROBOT_MAP_STATS = re.compile(r'<LocalMapStats>(.+?)</LocalMapStats>').finditer

def getProfitWrtEnvir(boardSupGridStats, expTargetsList, fileText):
    # Each robot will have a different envir as it calcs profit, as it will be taking into acc data that
    # it has just collected itself, => can't use global data for all robots
    robotEnvirStats = {}
    iter = FINDITER_ROBOT_MAP_STATS(fileText)
    for m in iter:
        statsText = m.group(1)
        statsDict = eval(pythonify(statsText))
        id = statsDict['iter']
        id = int(id)
        if id not in robotEnvirStats:
            robotEnvirStats[id] = {}
        robotEnvirStats[id]['nPixelsMapped'] = statsDict['nPixelsMapped']
        robotEnvirStats[id]['nExpCellsMapped'] = statsDict['nExpCellsMapped']
    
    # When looking at profit calculation, we should match this against the most recent stats, including
    # the current iteration

    profitWrtEnvirDict = {}
    iter = FINDITER_HIGH_LEVEL_BEH(fileText)
    for m in iter:
        matchStart = m.start()
        iterStart = fileText.rfind('<RobotStatus>', 0, matchStart)
        iterEnd = fileText.find('<RobotStatus>', matchStart)
        
        iteration = SEARCH_ITERATOR(fileText, iterStart, matchStart).group(1)
        iteration = int(iteration)
        
        supAreaMatch = re.search('supAreaId=(\(.+?\))', fileText[iterStart:iterEnd])
        if supAreaMatch:
            supAreaId = eval(supAreaMatch.group(1))
        else:
            supAreaId = None

        assert iteration not in profitWrtEnvirDict
        profitWrtEnvirDict[iteration] = {'exp': {}, 'gtepCollab': {}, 'closeLoop': {}, 'supAreaId': supAreaId}
        
        expMatch = SEARCH_EXP_PROFIT(fileText, iterStart, iterEnd)
        if expMatch:
            expText = expMatch.group(1)
            expDict = eval(pythonify(expText))
            profitWrtEnvirDict[iteration]['exp']['ratio'] = expDict['ratio']
            profitWrtEnvirDict[iteration]['exp']['gross'] = expDict['gross']
            #profitWrtEnvirDict[iteration]['exp']['mapGain_subd'] = mapGain_subd
            profitWrtEnvirDict[iteration]['exp']['resources'] = expDict['resources']
            profitWrtEnvirDict[iteration]['exp']['gainNMapped'] = expDict['gainNMapped']
            profitWrtEnvirDict[iteration]['exp']['nSteps'] = expDict['nSteps']

        gtepCollabMatch = SEARCH_GTEP_PROFIT_COLLAB(fileText, iterStart, iterEnd)
        if gtepCollabMatch:
            gtepCollabText = gtepCollabMatch.group(1)
            gtepCollabDict = eval(pythonify(gtepCollabText))
            profitWrtEnvirDict[iteration]['gtepCollab']['ratio'] = gtepCollabDict['ratio']
            profitWrtEnvirDict[iteration]['gtepCollab']['gross'] = gtepCollabDict['gross']
            profitWrtEnvirDict[iteration]['gtepCollab']['resources'] = gtepCollabDict['resources']

        closeLoopMatch = SEARCH_CLOSE_LOOP_PROFIT(fileText, iterStart, iterEnd)
        if closeLoopMatch:
            closeLoopText = closeLoopMatch.group(1)
            closeLoopDict = eval(pythonify(closeLoopText))
            profitWrtEnvirDict[iteration]['closeLoop']['ratio'] = closeLoopDict['profitRatio']
            profitWrtEnvirDict[iteration]['closeLoop']['gross'] = closeLoopDict['ownGross']
            profitWrtEnvirDict[iteration]['closeLoop']['resources'] = closeLoopDict['resources']

    #set_trace()
    for id in profitWrtEnvirDict:
        mostRecentRobotStats = [x for x in robotEnvirStats.keys() if x <= id]
        if not mostRecentRobotStats:
            print('WARNING: coundn\'t find robot envir stats for iter %d' % id)
            nPixelsMapped = 0
            nExpCellsMapped = 0
        else:
            robotStatsId = max(mostRecentRobotStats)
            nPixelsMapped = robotEnvirStats[robotStatsId]['nPixelsMapped']
            nExpCellsMapped = robotEnvirStats[robotStatsId]['nExpCellsMapped']

        if profitWrtEnvirDict[id]['supAreaId']:
            mostRecentBoardStats = [x for x in boardSupGridStats.keys() if x <= id]

            if not mostRecentBoardStats:
                print('WARNING: coundn\'t find board envir stats for iter %d' % id)
                supArea_nExpCellsMapped = 0
                supArea_nLocalMapsExhausted = 0
                supArea_nPixelsMapped = 0
            else:
                boardStatsId = max(mostRecentBoardStats)
                area = profitWrtEnvirDict[id]['supAreaId']
                supArea_nExpCellsMapped = boardSupGridStats[boardStatsId][area]['nExpCellsMapped']
                supArea_nLocalMapsExhausted = boardSupGridStats[boardStatsId][area]['nLocalMapsExhausted']
                supArea_nPixelsMapped = boardSupGridStats[boardStatsId][area]['nPixelsMapped']
        else:
            supArea_nExpCellsMapped = 0
            supArea_nLocalMapsExhausted = 0
            supArea_nPixelsMapped = 0

        profitWrtEnvirDict[id]['stats'] = {
            'nPixelsMapped': nPixelsMapped,
            'nExpCellsMapped': nExpCellsMapped,
            'supArea_nExpCellsMapped': supArea_nExpCellsMapped,
            'supArea_nLocalMapsExhausted': supArea_nLocalMapsExhausted,
            'supArea_nPixelsMapped': supArea_nPixelsMapped,
            }

    return profitWrtEnvirDict

def getGrossAttribdWrtAreaMapped(subdMapsOnRobot, boardSupGridStats, fileText):
    #area = (1, 0)  # (252, 126)
    area = (0, 1)
    print('HACK WARNING: only looking at board envir stats for area %s - used in grossAttribdWrtAreaMapped.csv' % str(area))
    
    subdMapDict = {x['iteration']:x for x in subdMapsOnRobot}
    maxIteration = max(max(subdMapDict.keys()), max(boardSupGridStats.keys()))

    grossWrtAreaMappedList = []
    lastIncdProfitIter = -1
    lastCellsMappedIter = -1
    lastCellsMappedValue = 0
    for iter in range(maxIteration + 1):
        incdProfit = None
        incdAvgStdDev = None
        incdMapGain = None
        nItersSinceLastProfit = None
        profitPerIterSinceLastProfit = None
        mapGainPerIterSinceLastProfit = None
        nExpCellsMapped = 0
        nItersSinceLastCellsMapped = None
        cellsSinceLastCellsMapped = None
        cellsPerIterSinceLastCellsMapped = None
        
        if iter in subdMapDict:
            incdProfit = subdMapDict[iter]['incdProfit']
            incdAvgStdDev = subdMapDict[iter]['avgStdDev']
            incdMapGain = subdMapDict[iter]['incdMapGain']
            nItersSinceLastProfit = iter - lastIncdProfitIter
            lastIncdProfitIter = iter
            profitPerIterSinceLastProfit = incdProfit / nItersSinceLastProfit
            mapGainPerIterSinceLastProfit = incdMapGain / nItersSinceLastProfit
        if iter in boardSupGridStats:
            nExpCellsMapped = boardSupGridStats[iter][area]['nExpCellsMapped']
            if nExpCellsMapped != lastCellsMappedValue:
                nItersSinceLastCellsMapped = iter - lastCellsMappedIter
                lastCellsMappedIter = iter
                cellsSinceLastCellsMapped = (nExpCellsMapped - lastCellsMappedValue)
                cellsPerIterSinceLastCellsMapped = float(cellsSinceLastCellsMapped) / nItersSinceLastCellsMapped
                lastCellsMappedValue = nExpCellsMapped
        tuple = (iter, incdProfit, incdAvgStdDev, incdMapGain, nItersSinceLastProfit, profitPerIterSinceLastProfit, mapGainPerIterSinceLastProfit, nExpCellsMapped, nItersSinceLastCellsMapped, cellsSinceLastCellsMapped, cellsPerIterSinceLastCellsMapped)
        grossWrtAreaMappedList.append(tuple)
    return grossWrtAreaMappedList

def getBehavioursAdopted(fileText, stdDevIncList):
    '''Extract <HighLevelBehavior> tags and search for the behaviour adopted.'''
    expTargets = []
    gtepTargets = []
    closeLoopTargets = []
    supTargets = []

    def __getCloseLoopScanBehaviours(fileText, startIndex, endIndex):
        '''
        Find the current scans against which close loop profit has been calculated. The scans are identified based on
        their position from the start of the map scan list, thus we can compare the values here, when the behaviour is
        adopted, with the same scans wheh loop closing has been completed and they are being submitted.
        '''
        scanBehaviours = {}
        
        scanIter = FINDITER_CLOSE_LOOP_SCANS(fileText, startIndex, endIndex)
        for scanMatch in scanIter:
            text = scanMatch.group(1)
            scanDict = eval(pythonify(text))
            scanIndex = int(scanDict['scan'])
            scanDict.pop('scan')
            scanBehaviours[scanIndex] = scanDict

        return scanBehaviours

    def __getSupInfo(fileText, startIndex, endIndex):
        infoDict = {}
        propMatch = SEARCH_SUP_PROFIT(fileText, startIndex, endIndex)
        propDict = eval(pythonify(propMatch.group(1)))
        infoDict['stdDev'] = propDict['stdDevAtDest']
        #infoDict['nExpCells'] = propDict['proposalExpCellsMapped']
        coalMatch = SEARCH_SUP_COAL(fileText, startIndex, endIndex)

        #
        # TODO: handle supervisor with multiple partners properly
        #
        #try:
        #set_trace()
        coalText = coalMatch.group(1)
        coalTuples = coalText.split('),(')
        def __fixCoal(coalText):
            if coalText.endswith('),'):
                coalText = coalText[:-2]
            else:
                coalText = coalText[:-1]
            return coalText[1:]

        coalTuples = [__fixCoal(x) for x in coalTuples]
        coalDict = eval(pythonify(coalTuples[0]))
        #except:
        #    set_trace()
        infoDict['coalId'] = coalDict['id']
        return infoDict

    def __getCloseLoopInfo(fileText, startIndex, endIndex):
        infoMatch = SEARCH_CLOSE_LOOP_PROFIT_INFO(fileText, startIndex, endIndex)
        infoText = infoMatch.group(1)
        infoText = pythonify(infoText)
        infoDict = eval(infoText)
        return infoDict
    
    behIter = FINDITER_HIGH_LEVEL_BEH(fileText)
    for behMatch in behIter:
        behIndex = behMatch.start()
        startIndex = fileText.rfind('<RobotStatus>', 0, behIndex)
        endIndex = fileText.find('<RobotStatus>', behIndex)
        iteration = SEARCH_ITERATOR(fileText, startIndex, behIndex).group(1)
        iteration = int(iteration)
        #try:
        localMapIndex = SEARCH_LOC_MAP_INDEX(fileText, startIndex, behIndex).group(1)
        #except:
        #    set_trace()
        localMapIndex = int(localMapIndex)
        #set_trace()
        actLocOffset = SEARCH_ACT_LOC_OFFSET(fileText, startIndex, behIndex).group(1)
        actLocOffset = eval(actLocOffset)
        actLocOffset = math.sqrt(actLocOffset[0] * actLocOffset[0] + actLocOffset[1] * actLocOffset[1])

        # Search from behIndex here, as we may have '<AdoptBehaviour>behaviour="AVAILABLE"' from
        # checkNavigationState, and then carry on to adopt a proper behaviour in highLevelBehaviour
        adoptMatch = SEARCH_ADOPT_BEH(fileText, behIndex, endIndex) 
        if not adoptMatch:
            continue

        adoptText = adoptMatch.group(1)
        adoptText = pythonify(adoptText)
        adoptDict = eval(adoptText)
        if adoptDict['behaviour'] not in ('EXPLORATION', 'GOTO_EXP_PT', 'GOTO_EXP_PT_COLLAB', 'CLOSE_LOOP', 'SUPERVISION'):
            continue
        if adoptDict['behaviour'] == 'EXPLORATION':
            behProfitMatch = SEARCH_EXP_PROFIT(fileText, startIndex, endIndex)
        elif adoptDict['behaviour'] == 'GOTO_EXP_PT':
            behProfitMatch = SEARCH_GTEP_PROFIT(fileText, startIndex, endIndex)
        elif adoptDict['behaviour'] == 'GOTO_EXP_PT_COLLAB':
            behProfitMatch = SEARCH_GTEP_PROFIT_COLLAB(fileText, startIndex, endIndex)
        elif adoptDict['behaviour'] == 'SUPERVISION':
            behProfitMatch = SEARCH_SUP_PROFIT(fileText, startIndex, endIndex)
        else:
            behProfitMatch = SEARCH_CLOSE_LOOP_PROFIT(fileText, startIndex, endIndex)
        behProfitText = behProfitMatch.group(1)
        behProfitText = pythonify(behProfitText)
        behProfitDict = eval(behProfitText)
        behProfitDict['iteration'] = iteration
        behProfitDict['actLocOffset'] = actLocOffset
        behProfitDict['localMapIndex'] = localMapIndex
        behProfitDict['targetIndex'] = adoptDict['targetIndex']
        if adoptDict['behaviour'] == 'EXPLORATION':
            expTargets.append(behProfitDict)
        elif adoptDict['behaviour'] in ['GOTO_EXP_PT', 'GOTO_EXP_PT_COLLAB']:
            behProfitDict['potentialGtepSessionId'] = adoptDict['potentialGtepSessionId']
            gtepTargets.append(behProfitDict)
        elif adoptDict['behaviour'] == 'SUPERVISION':
            behProfitDict.update(__getSupInfo(fileText, startIndex, endIndex))
            supTargets.append(behProfitDict)
        else:
            behProfitDict['scanBehaviours'] = __getCloseLoopScanBehaviours(fileText, startIndex, endIndex)
            behProfitDict['additionalInfo'] = __getCloseLoopInfo(fileText, startIndex, endIndex)
            closeLoopTargets.append(behProfitDict)

    for expTarget in expTargets:
        stdDevTuples = [x for x in stdDevIncList if x['target'] == expTarget['targetIndex']]
        expTarget['stdDevInc_actual'] = sum(x['after'] - x['before'] for x in stdDevTuples)
        expTarget['nScans_actual_redundant'] = len(stdDevTuples)

    for gtepTarget in gtepTargets:
        stdDevTuples = [x for x in stdDevIncList if x['target'] == gtepTarget['targetIndex']]
        gtepTarget['stdDevInc_actual'] = sum(x['after'] - x['before'] for x in stdDevTuples)
        gtepTarget['nScans_actual_redundant'] = len(stdDevTuples)
    
    return (expTargets, gtepTargets, closeLoopTargets, supTargets)


def extractResultsFromExperiments(experimentDirs):
    '''Iterate through experiment directories and build dictionary of results for each.''' 
    resultsDictList = []
    
    for (boardFile, robotFiles, outputDir) in experimentDirs:
        resultsDict = {}
        incdMapsOnBoard = getIncdMapsOnBoard(boardFile)

        boardSupGridStats = getSupGridsStatsOnBoard(boardFile)

        resultsDict['globalMapDetails'] = getGlobalMapDetails(boardFile)
        resultsDict['globalMapDetails']['nRobots'] = len(robotFiles)
        resultsDict['globalMapDetails']['nLocalMaps'] = len(incdMapsOnBoard)
        resultsDict['globalMapDetails']['errorPerMoveList'] = []
        resultsDict['globalMapDetails']['moveList'] = []
        resultsDict['globalMapDetails']['coalInfo'] = {}

        maxIteration = 0
        #for robotFile in robotFiles[1:]:
        for robotFile in robotFiles:
            fileText = open(robotFile).read()
            robotIndex = int(robotFile.split('_')[-2])

            sanityCheckFile(fileText)
            moveList, lastIteration, errorPerMove = getRobotMoves(fileText)

            offsets = getOffsets(fileText)
            mapScanList = getMapScans(fileText)
            stdDevIncList = getStdDevIncs(fileText)
            subdMapsOnRobot, closeLoopSessions = getSubdMapsOnRobot(fileText)
            coalitionInfo = getCoalitionInfo(fileText)
            expTargetList, gtepTargetList, closeLoopTargetList, supTargetList = getBehavioursAdopted(fileText, stdDevIncList)
            matchSubdAndIncdMaps(robotIndex, incdMapsOnBoard, subdMapsOnRobot)
            matchCloseLoopSessions(closeLoopSessions, closeLoopTargetList)
            profitWrtEnvirDict = getProfitWrtEnvir(boardSupGridStats, expTargetList, fileText)
            grossWrtAreaMapped = getGrossAttribdWrtAreaMapped(subdMapsOnRobot, boardSupGridStats, fileText)
            
            resultsDict['offsetList_%d' % robotIndex] = offsets
            resultsDict['subdMapList_%d' % robotIndex] = subdMapsOnRobot
            resultsDict['closeLoopSessions_%d' % robotIndex] = closeLoopSessions
            resultsDict['coalitionInfo_%d' % robotIndex] = coalitionInfo
            resultsDict['expTargetList_%d' % robotIndex] = expTargetList
            resultsDict['gtepTargetList_%d' % robotIndex] = gtepTargetList
            resultsDict['supTargetList_%d' % robotIndex] = supTargetList
            resultsDict['profitWrtEnvir_%d' % robotIndex] = profitWrtEnvirDict
            resultsDict['grossWrtAreaMapped_%d' % robotIndex] = grossWrtAreaMapped
            resultsDict['globalMapDetails']['errorPerMoveList'].append(errorPerMove)
            resultsDict['globalMapDetails']['moveList'].extend(moveList)
            maxIteration = max(maxIteration, lastIteration)

        resultsDict['globalMapDetails']['nIterations'] = maxIteration
        resultsDictList.append(resultsDict)

    return resultsDictList
