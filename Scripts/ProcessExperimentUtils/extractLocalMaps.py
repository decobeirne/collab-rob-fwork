import re
import extractResults
from pdb import set_trace


FINDITER_SUB_LOCAL_MAP = re.compile('<SubmitLocalMap>(.+?)</SubmitLocalMap>').finditer
SEARCH_WRITE_LOCAL_MAP = re.compile('<WriteLocalMap>(.+?)</WriteLocalMap>').search
SEARCH_SERIALIZE_LOCAL_MAP = re.compile('<SerializedLocalMap>(.+?)</SerializedLocalMap>').search
SEARCH_ITERATION = re.compile('iteration=(.+?) ').search
def getMapsFromLog(robotLogName):
    print('Extracting local maps from log file %s' % robotLogName)
    robotLog = open(robotLogName, 'rb')
    robotLogText = robotLog.read()
    assert robotLog
    localMapList = []
    for localMapMatch in FINDITER_SUB_LOCAL_MAP(robotLogText):
        startLoc = localMapMatch.start()
        endLoc = localMapMatch.end()
        iterStartLoc = robotLogText.rfind('<RobotStatus>', 0, startLoc)
        iterEndLoc = robotLogText.find('<RobotStatus>', endLoc)
        localMapDict = eval(extractResults.pythonify(localMapMatch.group(1)))
        serializeMatch = SEARCH_SERIALIZE_LOCAL_MAP(robotLogText[iterStartLoc:iterEndLoc])
        serializeDict = eval(extractResults.pythonify(serializeMatch.group(1)))
        writeMatch = SEARCH_WRITE_LOCAL_MAP(robotLogText[iterStartLoc:iterEndLoc])
        writeDict = eval(extractResults.pythonify(writeMatch.group(1)))
        iterationMatch = SEARCH_ITERATION(robotLogText[iterStartLoc:iterEndLoc])
        iteration = int(iterationMatch.group(1))

        outputDict = {}
        outputDict['robotIndex'] = localMapDict['robotIndex']
        outputDict['localMapIndex'] = localMapDict['localMapIndex']
        outputDict['isProvisional'] = localMapDict['isProvisional']
        outputDict['closeLoopSession'] = localMapDict['closeLoopSession']
        outputDict['filename'] = serializeDict['filename']
        outputDict['origin'] = writeDict['origin']
        outputDict['iteration'] = iteration
        #outputDict[] = localMapDict[]
        #outputDict[] = localMapDict[]
        localMapList.append(outputDict)
    return localMapList
    
def populateStructsForReplaying(localMaps, outputFilename):
    print('Writing local maps to code in %s' % outputFilename)
    outputFile = open(outputFilename, 'w')
    assert outputFile

    def write(outputStr):
        outputFile.write(outputStr)
        outputFile.write('\n')

    # Regarding isMoreDataComing - this would be used to tell the board not to do loads of processing
    # when the robot is about to submit more local maps straight away afterwards, but the end result will
    # not be effected if we do the processing every time, so actually would be better to ignore isMoreDataComing,
    # so that we have the option of displaying a snap shot of the global map as each map is integrated.

    write('nMaps = %d;' % len(localMaps))
    index = 0
    for localMap in localMaps:
        write('localMaps[%d].robotIndex = %d;' % (index, localMap['robotIndex']))
        write('localMaps[%d].localMapIndex = %d;' % (index, localMap['localMapIndex']))
        write('localMaps[%d].isProvisional = %d;' % (index, localMap['isProvisional']))
        write('localMaps[%d].closeLoopSession = %d;' % (index, localMap['closeLoopSession']))
        write('strcpy (localMaps[%d].filename, "%s");' % (index, localMap['filename']))
        write('localMaps[%d].origin.x = %d;' % (index, localMap['origin'][0]))
        write('localMaps[%d].origin.y = %d;' % (index, localMap['origin'][1]))
        write('localMaps[%d].iteration = %d;' % (index, localMap['iteration']))
        write('localMaps[%d].isMoreDataComing = %d;' % (index, 0))
        write('')
        index += 1