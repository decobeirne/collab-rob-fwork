from pdb import set_trace
import time
import os
import pprint
2948.4






def createResultsDir():
    lt = time.localtime()
    timeStr = 'Results_%d%02d%02d_%02d%02d%02d' % (lt.tm_year, lt.tm_mon, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec)
    os.mkdir(timeStr)
    return timeStr

def printGlobalMaps(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFile = None

    for resultsDict in resultsDictList:
        if not fileSetup:
            fileSetup = True
            outputFile = open(os.path.join(resultsDir, 'globalMaps.csv'), 'w')
            globalMap = resultsDict['globalMapDetails']
            globalMapKeys = sorted(list(globalMap.keys()))
            globalMapKeys.remove('errorPerMoveList')  # Don't print.
            globalMapKeys.remove('moveList')  # Don't print.
            globalMapKeys.remove('coalInfo')  # Don't print.
            outputFile.write(','.join(globalMapKeys) + '\n')

        globalMap = resultsDict['globalMapDetails']
        globalMapVals = [globalMap[x] for x in globalMapKeys]
        outputFile.write(','.join(str(x) for x in globalMapVals) + '\n')
    outputFile.close()

def printOffsets(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFile = None

    listOfRobotOffsets = []
    maxOffsets = 0
    
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'offsetList_%d' % index not in resultsDict:
                index += 1
                continue

            if not fileSetup:
                fileSetup = True
                outputFile = open(os.path.join(resultsDir, 'offsets.csv'), 'w')
            
            offsetList = resultsDict['offsetList_%d' % index]
            nOffsets = len(offsetList)
            if nOffsets > maxOffsets:
                maxOffsets = nOffsets

            offsetTuple = (index, offsetList)
            listOfRobotOffsets.append(offsetTuple)
            
            index += 1

    ids = [str(x[0]) for x in listOfRobotOffsets]
    outputFile.write(','.join(ids) + '\n')

    for i in range(maxOffsets):
        textList = []
        for robotId, offsetList in listOfRobotOffsets:

            if len(offsetList) > i:
                textList.append(str(offsetList[i]))
            else:
                textList.append('')
        outputFile.write(','.join(textList) + '\n')

            

'''
            
            
            index += 1
            
    if fileSetup:
        outputFile.close()

    nRobots = len(resultsDictList)
    
    outputFile = open(os.path.join(resultsDir, 'offsets.csv'), 'w')
    
    titleList = ['Robot %d' % i for i in range(nRobots)]
    outputFile.write(','.join(titleList) + '\n')
    
    listOfLists = []
    index = 0
    maxOffsets = 0
    
    set_trace()
    for resultsDict in resultsDictList:
        offsetList = resultsDict['offsetList_%d' % index]
        nOffsets = len(offsetList)
        if nOffsets > maxOffsets:
            maxOffsets = nOffsets
        listOfLists.append(offsetList)
        index += 1

    for i in range(maxOffsets):
        textList = []
        for offsetList in listOfLists:
            if len(offsetList) > i:
                textList.append(str(offsetList[i]))
            else:
                textList.append('')
        outputFile.write(','.join(textList) + '\n')
        
    outputFile.close()'''

def printLocalMaps(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFile = None
    
    def __removeKeys(keysList, keysToRemove):
        for key in keysToRemove:
            try:
                keysList.remove(key)
            except:
                pass

    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'subdMapList_%d' % index not in resultsDict:
                index += 1
                continue
            
            if not fileSetup:
                fileSetup = True
                outputFile = open(os.path.join(resultsDir, 'localMaps.csv'), 'w')
                subdMap = resultsDict['subdMapList_%d' % index][0]
                subdMapKeys = sorted(list(subdMap.keys()))
                __removeKeys(subdMapKeys, ['behaviourProfits', 'targetProfits', 'profitRatioList', 'profitToCoal'])
                outputFile.write(','.join(subdMapKeys) + '\n')
                
            subdMapList = resultsDict['subdMapList_%d' % index]
            for subdMap in subdMapList:
                subdMapVals = [subdMap[x] for x in subdMapKeys]
                outputFile.write(','.join(str(x) for x in subdMapVals) + '\n')
            index += 1
    outputFile.close()

def printExpTargets(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFile = None
    
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'expTargetList_%d' % index not in resultsDict or not resultsDict['expTargetList_%d' % index]:
                index += 1
                continue
            
            if not fileSetup:
                fileSetup = True
                outputFile = open(os.path.join(resultsDir, 'expTargets.csv'), 'w')
                expTarget = resultsDict['expTargetList_%d' % index][0]
                expTargetKeys = sorted(list(expTarget.keys()))
                expTargetKeys.remove('target')  # Don't print.
                outputFile.write(','.join(expTargetKeys) + '\n')
            
            expTargetList = resultsDict['expTargetList_%d' % index]
            for expTarget in expTargetList:
                expTargetVals = [expTarget[x] for x in expTargetKeys]
                outputFile.write(','.join(str(x) for x in expTargetVals) + '\n')

            index += 1
    outputFile.close()

def printGtepTargets(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFile = None
    
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'gtepTargetList_%d' % index not in resultsDict or not resultsDict['gtepTargetList_%d' % index]:
                index += 1
                continue

            if not fileSetup:
                fileSetup = True
                outputFile = open(os.path.join(resultsDir, 'gtepTargets.csv'), 'w')
                gtepTarget = resultsDict['gtepTargetList_%d' % index][0]
                gtepTargetKeys = sorted(list(gtepTarget.keys()))
                gtepTargetKeys.remove('target')  # Don't print.
                gtepTargetKeys.remove('allocatedProfitList')
                outputFile.write(','.join(gtepTargetKeys) + '\n')      

            gtepTargetList = resultsDict['gtepTargetList_%d' % index]
            for gtepTarget in gtepTargetList:
                gtepTargetVals = [gtepTarget[x] for x in gtepTargetKeys]
                outputFile.write(','.join(str(x) for x in gtepTargetVals) + '\n')

            index += 1
    if outputFile:
        outputFile.close()

def printSupTargets(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFile = None
    
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'supTargetList_%d' % index not in resultsDict or not resultsDict['supTargetList_%d' % index]:
                index += 1
                continue

            if not fileSetup:
                fileSetup = True
                outputFile = open(os.path.join(resultsDir, 'supTargets.csv'), 'w')
                #outputFile.write('iteration,ratio,gross,resources,proposalStdDev,proposalExpCellsMapped,nAllocs,profitAllocd,nSubdMaps,profitSubdMaps,\n')
                outputFile.write('iteration,ratio,gross,resources,proposalStdDev,nAllocs,profitAllocd,nSubdMaps,profitSubdMaps,\n')
            supTargetList = resultsDict['supTargetList_%d' % index]
            for supTarget in supTargetList:
                #outputFile.write('%d,%f,%f,%f,%f,%f,%d,%f,%d,%f,\n' % (supTarget['iteration'], supTarget['ratio'], supTarget['gross'], supTarget['resources'], supTarget['proposalStdDev'], supTarget['proposalExpCellsMapped'], supTarget['nAllocs'], supTarget['profitAllocd'], supTarget['nSubdMaps'], supTarget['profitSubdMaps']))
                outputFile.write('%d,%f,%f,%f,%f,%d,%f,%d,%f,\n' % (supTarget['iteration'], supTarget['ratio'], supTarget['gross'], supTarget['resources'], supTarget['proposalStdDev'], supTarget['nAllocs'], supTarget['profitAllocd'], supTarget['nSubdMaps'], supTarget['profitSubdMaps']))
                #set_trace()
                pass
            index += 1
    if outputFile: outputFile.close()

def printCloseLoopSessions(resultsDictList, dirs, resultsDir):
    fileSetup = False
    outputFileSessions = None
    outputFileScans = None
    outputFileScans2 = None
    scanAdjsList = []
    
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'closeLoopSessions_%d' % index not in resultsDict:
                index += 1
                continue
            
            if not fileSetup:
                fileSetup = True
                outputFileSessions = open(os.path.join(resultsDir, 'closeLoopSessions.csv'), 'w')
                closeLoopSessionKeys = [
                    'iterationAdopting',
                    'iterationSubmitting',
                    'nScansInitial',
                    'nScansTotalWhenAdopting',
                    'nScansTotalWhenSubmitting',
                    'nScansLoopClosingWhenAdopting',
                    'nScansLoopClosingWhenSubmitting',
                    'nScansAdjustedWhenAdopting',
                    'nScansAdjustedInitialWhenAdopting',
                    'nScansAdjustedWhenSubmitting',
                    'lastStdDevWhenAdopting',
                    'lastStdDevWhenSubmitting',
                    'grossAdjsWhenAdopting',
                    'grossNewScansWhenAdopting',
                    'grossAdjsWhenSubmitting',
                    'grossNewScansWhenSubmitting',
                    'scanAdjsAvgIncOffset',
                    'scanAdjsNormdIncOffset',
                    'scanAdjsAvgIncStdDev',
                    ]
                outputFileSessions.write(','.join(closeLoopSessionKeys) + '\n')      

                outputFileScans = open(os.path.join(resultsDir, 'closeLoopScans.csv'), 'w')
                closeLoopScanKeys = [
                    'scanIndex',
                    'offsetOrig',
                    'offsetAdj',
                    'origStdDev',
                    'adjStdDev',
                    ]
                outputFileScans.write(','.join(closeLoopScanKeys) + ',diff' + '\n')
                closeLoopScanKeys.pop(0)

            closeLoopSessionList = resultsDict['closeLoopSessions_%d' % index]
            for session in closeLoopSessionList:
                sessionVals = [session[x] for x in closeLoopSessionKeys]
                outputFileSessions.write(','.join(str(x) for x in sessionVals) + '\n')
                scanAdjs = session['scanAdjustments']
                scanAdjsList.append(scanAdjs)
                for scanIndex in scanAdjs:
                    scanAdj = scanAdjs[scanIndex]
                    scanVals = [scanAdj.get(x, '') for x in closeLoopScanKeys]
                    outputFileScans.write('%d,' % scanIndex)
                    diff = ''
                    if 'offsetOrig' in scanAdj and 'offsetAdj' in scanAdj:
                        diff = scanAdj['offsetOrig'] - scanAdj['offsetAdj']
                    scanVals.append(diff)
                    outputFileScans.write(','.join(str(x) for x in scanVals) + '\n')

            index += 1

    # Need to print scan adjustments per iteration in order to determine typical range or average
    # standard deviation of sub map scans relative to a supervisor standard deviation in a coalition
    #
    # This is necessary in estimating the utility of the CLOSE_LOOP profits
    outputFileScans2 = open(os.path.join(resultsDir, 'closeLoopScans2.csv'), 'w')
    nSessions = len(scanAdjsList)
    closeLoopScanKeys2 = ['scanIndex']
    closeLoopScanKeys2.extend(['offsetSession%d' % x for x in range(nSessions)])
    closeLoopScanKeys2.extend(['diffSession%d' % x for x in range(nSessions)])
    closeLoopScanKeys2.extend(['stdDevSession%d' % x for x in range(nSessions)])
    outputFileScans2.write(','.join(closeLoopScanKeys2) + '\n')
    maxIndices = [max(x.keys()) for x in scanAdjsList]
    maxIndex = max(maxIndices) if maxIndices else 0

    for scanIndex in range(maxIndex + 1):
        offsets = []
        stdDevs = []
        diffs = []
        for scanAdjs in scanAdjsList:
            offsetAdj = ''
            stdDev = ''
            diff = ''
            if scanIndex in scanAdjs:
                # scanAdjsList is resultsDict['closeLoopSessions_0][i]['scanAdjustments']
                # These are set in getSubdMapsOnRobot()
                # Each scanAdj is just eval'd from a <AdjustScan>(.+?)</AdjustScan>
                scanAdj = scanAdjs[scanIndex]
                offsetAdj = scanAdj.get('offsetAdj', '')
                stdDev = scanAdj.get('adjStdDev', '')
                offsetOrig = scanAdj.get('offsetOrig', '')
                if offsetAdj and offsetOrig:
                    diff = offsetOrig - offsetAdj
            offsets.append(offsetAdj)
            diffs.append(diff)
            stdDevs.append(stdDev)
        outputFileScans2.write('%d,%s,%s,%s,\n' % (scanIndex, ','.join(str(x) for x in offsets), ','.join(str(x) for x in diffs), ','.join(str(x) for x in stdDevs)))

    if outputFileSessions: outputFileSessions.close()
    if outputFileScans: outputFileScans.close()
    if outputFileScans2: outputFileScans2.close()


def printProfitWrtEnvir(resultsDictList, dirs, resultsDir):
    fileSetup = False
    expProfitWrtEnvir = None
    gtepcProfitWrtEnvir = None
    closeLoopProfitWrtEnvir = None
    profitRatiosWrtEnvir = None
    
    #set_trace()
    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'profitWrtEnvir_%d' % index not in resultsDict:
                index += 1
                continue
            
            if not fileSetup:
                fileSetup = True
                expProfitWrtEnvir = open(os.path.join(resultsDir, 'expProfitWrtEnvir.csv'), 'w')
                gtepcProfitWrtEnvir = open(os.path.join(resultsDir, 'gtepcProfitWrtEnvir.csv'), 'w')
                closeLoopProfitWrtEnvir = open(os.path.join(resultsDir, 'closeLoopProfitWrtEnvir.csv'), 'w')
                keysToPrint = [
                    'iteration',
                    'nPixelsMapped',
                    'nExpCellsMapped',
                    'supArea_nExpCellsMapped',
                    'supArea_nLocalMapsExhausted',
                    'supArea_nPixelsMapped',
                    'ratio',
                    'gross',
                    'resources',
                    'mapGain',
                    'nSteps',
                    'mapGain_subd',
                    ]
                expProfitWrtEnvir.write(','.join(keysToPrint) + '\n')
                gtepcProfitWrtEnvir.write(','.join(keysToPrint) + '\n')
                closeLoopProfitWrtEnvir.write(','.join(keysToPrint) + '\n')

                profitRatiosWrtEnvir = open(os.path.join(resultsDir, 'profitRatios.csv'), 'w')
                #profitRatiosWrtEnvir.write('expRatioWrtExpCellsMapped,gtepRatioWrtLocalMapsExhausted,\n')
                profitRatiosWrtEnvir.write('expRatioWrtExpCellsMapped,gtepRatioWrtExpCellsMapped,\n')
                expRatioList = []
                gtepcRatioList = []
            
            profitWrtEnvirDict = resultsDict['profitWrtEnvir_%d' % index]
            iterations = sorted(profitWrtEnvirDict.keys())
            for iteration in iterations:
                statsDict = profitWrtEnvirDict[iteration]['stats']
                expRatio = ''
                gtepcRatio = ''
                if profitWrtEnvirDict[iteration]['exp']:
                    expDict = profitWrtEnvirDict[iteration]['exp']
                    expProfitWrtEnvir.write('%d,' % (iteration))
                    expProfitWrtEnvir.write('%f,%f,%f,%f,%f,' % (statsDict['nPixelsMapped'], statsDict['nExpCellsMapped'], statsDict['supArea_nExpCellsMapped'], statsDict['supArea_nLocalMapsExhausted'], statsDict['supArea_nPixelsMapped']))
                    #set_trace()
                    expProfitWrtEnvir.write('%f,%f,%f,%f,%d,%d\n' % (expDict['ratio'], expDict['gross'], expDict['resources'], expDict['gainNMapped'], expDict['nSteps'], expDict['mapGain_subd']))
                    expRatio = expDict['ratio'] / max(1, statsDict['nExpCellsMapped'])
                    expRatioList.append(expRatio)
                
                if profitWrtEnvirDict[iteration]['gtepCollab']:
                    gtepcDict = profitWrtEnvirDict[iteration]['gtepCollab']
                    gtepcProfitWrtEnvir.write('%d,' % (iteration))
                    gtepcProfitWrtEnvir.write('%f,%f,%f,%f,%f,' % (statsDict['nPixelsMapped'], statsDict['nExpCellsMapped'], statsDict['supArea_nExpCellsMapped'], statsDict['supArea_nLocalMapsExhausted'], statsDict['supArea_nPixelsMapped']))
                    gtepcProfitWrtEnvir.write('%f,%f,%f,"","",\n' % (gtepcDict['ratio'], gtepcDict['gross'], gtepcDict['resources']))
                    
                    gtepcRatio = gtepcDict['ratio'] / max(1, statsDict['supArea_nExpCellsMapped'])
                    gtepcRatioList.append(gtepcRatio)

                if profitWrtEnvirDict[iteration]['closeLoop']:
                    clDict = profitWrtEnvirDict[iteration]['closeLoop']
                    closeLoopProfitWrtEnvir.write('%d,' % (iteration))
                    closeLoopProfitWrtEnvir.write('%f,%f,%f,%f,%f,' % (statsDict['nPixelsMapped'], statsDict['nExpCellsMapped'], statsDict['supArea_nExpCellsMapped'], statsDict['supArea_nLocalMapsExhausted'], statsDict['supArea_nPixelsMapped']))
                    closeLoopProfitWrtEnvir.write('%f,%f,%f,"","",\n' % (clDict['ratio'], clDict['gross'], clDict['resources']))
                
                profitRatiosWrtEnvir.write('%s,%s,\n' % (str(expRatio), str(gtepcRatio)))
            index += 1

    if expProfitWrtEnvir: expProfitWrtEnvir.close()
    if gtepcProfitWrtEnvir: gtepcProfitWrtEnvir.close()
    if closeLoopProfitWrtEnvir: closeLoopProfitWrtEnvir.close()
    if profitRatiosWrtEnvir: profitRatiosWrtEnvir.close()


def printGrossAttribdWrtAreaMapped(resultsDictList, dirs, resultsDir):
    fileSetup = False
    grossAttribdWrtAreaMapped = None

    for resultsDict in resultsDictList:
        index = 0
        MAX_INDEX = 20
        while index < MAX_INDEX:
            if 'grossWrtAreaMapped_%d' % index not in resultsDict:
                index += 1
                continue
            
            if not fileSetup:
                fileSetup = True
                grossAttribdWrtAreaMapped = open(os.path.join(resultsDir, 'grossAttribdWrtAreaMapped.csv'), 'w')
                grossAttribdWrtAreaMapped.write('iteration,profitIncd,incdAvgStdDev,incdMapGain,nItersSinceLastProfit,profitPerIterSinceLastProfit,mapGainPerIterSinceLastProfit,nExpCellsMapped,nItersSinceLastCellsMapped,cellsSinceLastCellsMapped,cellsPerIterSinceLastCellsMapped\n')

            grossList = resultsDict['grossWrtAreaMapped_%d' % index]
            #for (iter, incdProfit, nExpCellsMapped) in grossList:
            for (iter, incdProfit, incdAvgStdDev, incdMapGain, nItersSinceLastProfit, profitPerIterSinceLastProfit, mapGainPerIterSinceLastProfit, nExpCellsMapped, nItersSinceLastCellsMapped, cellsSinceLastCellsMapped, cellsPerIterSinceLastCellsMapped) in grossList:

                output = '%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,\n' % (
                    iter,
                    repr(incdProfit) if incdProfit else '',
                    repr(incdAvgStdDev) if incdAvgStdDev else '',
                    repr(incdMapGain) if incdMapGain else '',
                    repr(nItersSinceLastProfit) if nItersSinceLastProfit else '',
                    repr(profitPerIterSinceLastProfit) if profitPerIterSinceLastProfit else '',
                    repr(mapGainPerIterSinceLastProfit) if mapGainPerIterSinceLastProfit else '',
                    repr(nExpCellsMapped) if nExpCellsMapped else repr(0),
                    repr(nItersSinceLastCellsMapped) if nItersSinceLastCellsMapped else '',
                    repr(cellsSinceLastCellsMapped) if nItersSinceLastCellsMapped else '',
                    repr(cellsPerIterSinceLastCellsMapped) if nItersSinceLastCellsMapped else '',
                    )
                grossAttribdWrtAreaMapped.write(output)
            index += 1
    if grossAttribdWrtAreaMapped: grossAttribdWrtAreaMapped.close()

def printResultsToCsvFiles(resultsDictList, dirs):
    resultsDir = createResultsDir()
    csvFiles = {}
    
    printOffsets(resultsDictList, dirs, resultsDir)
    printGlobalMaps(resultsDictList, dirs, resultsDir)
    printLocalMaps(resultsDictList, dirs, resultsDir)
    printExpTargets(resultsDictList, dirs, resultsDir)
    printGtepTargets(resultsDictList, dirs, resultsDir)
    printSupTargets(resultsDictList, dirs, resultsDir)
    printCloseLoopSessions(resultsDictList, dirs, resultsDir)
    printProfitWrtEnvir(resultsDictList, dirs, resultsDir)
    printGrossAttribdWrtAreaMapped(resultsDictList, dirs, resultsDir)
    
    readmeFile = open(os.path.join(resultsDir, 'readme.txt'), 'w')
    pprint.pprint(dirs, readmeFile)
    readmeFile.write('\n\n')
    readmeFile.close()
    
    if 1:
        if len(dirs) == 1:
            
            import shutil
            destDir = os.path.normpath(os.path.join('../Files/', dirs[0]))
            shutil.move(resultsDir, destDir)
            print('Moving %s to %s' % (resultsDir, destDir))
        else:
            print('Not moving dirs to exper directories, as multiple directories')
    
    print('Results printed to %s' % resultsDir)
    
