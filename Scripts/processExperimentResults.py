import os
import re
import math
import pprint
import geometryUtils
from ProcessExperimentUtils import extractResults
from ProcessExperimentUtils import analyseResults
from ProcessExperimentUtils import printResults
from ProcessExperimentUtils import extractLocalMaps
from pdb import set_trace


def gatherExperimentFiles():
    dirs = []
    #dirs.append((r'experiment_robot_0_2481', r'experiment_board_1312738755'))
    #dirs.append((r'experiment_robot_0_2618', r'experiment_board_1312738893'))
    #dirs.append((r'experiment_robot_0_2784', r'experiment_board_1312739064'))
    dirs.append((r'experiment_robot_0_1005-19691231-Wed-161645', r'experiment_board_1338061655-20120526-Sat-204735'))
    dirs.append((r'experiment_robot_0_1310-19691231-Wed-162150', r'experiment_board_1338061929-20120526-Sat-205209'))
    dirs.append((r'experiment_robot_0_1714-19691231-Wed-162834', r'experiment_board_1338062363-20120526-Sat-205923'))

    #relDir = os.path.join(os.getcwd(), '../Files/')
    relDir = os.path.join(os.getcwd(), '../Files/20120526_pcGumstixExperiments')
    fileTuples = []
    for robotDir, boardDir in dirs:
        files = os.listdir(os.path.join(relDir, robotDir))
        robotFiles = [os.path.normpath(os.path.join(relDir, robotDir, x)) for x in files if x.startswith('robot_') or x.startswith('gum_')]
        files = os.listdir(os.path.join(relDir, boardDir))
        boardFile = [os.path.normpath(os.path.join(relDir, boardDir, x)) for x in files if x.startswith('board_')][0]

        fileTuple = (boardFile, robotFiles, os.path.join(relDir, boardDir))
        fileTuples.append(fileTuple)
    return fileTuples, dirs


def gatherSimFiles():
    '''Get results from simulation files. Adjust bool values to specify which to process.'''
    dirs = []
    relDir = os.path.normpath(os.path.join(os.getcwd(), '../Files/'))

    allSims = 0
    specifyList = 0
    lastSim = 1
    specifyRange = 0

    if allSims:
        expDirs = [x for x in os.listdir(relDir) if '_sim_' in x]
        expDirs = sorted(expDirs)
        dirs.extend(expDirs)

    if specifyList:
        #dirs.append(r'experiment_simulation_1315689110')
        #dirs.append(r'experiment_simulation_1315689154')
        #dirs.append(r'experiment_simulation_1315689188')
        #dirs.append(r'experiment_sim_1375775344-20130806-Tue-084904__1rob')
        #dirs.append(r'experiment_sim_1375775640-20130806-Tue-085400__2robs__indep__tookScreenShots')
        #dirs.append(r'experiment_sim_1375775873-20130806-Tue-085753__3robs__indep__screenShots')
        #dirs.append(r'experiment_sim_1375776058-20130806-Tue-090058__4robs__indep__screenShots')
        dirs.append(r'experiment_sim_1380233835-20130926-Thu-231715__c8robs__err40__nolim')
        #dirs.append(r'')
        print('Processing an explicitly selected set of dirs...')

    if lastSim:
        expDirs = [x for x in os.listdir(relDir) if '_sim_' in x]
        expDirs = sorted(expDirs)
        lastSimDir = expDirs[-1]
        dirs.append(lastSimDir)

    if specifyRange:
        #startTime = '1340053920'  # Orig exp approach
        #endTime = '1340053980'
        #startTime = '1340265300'  # Exp with more accurate map gain calc'ion (no improvement really)
        #endTime = '1340265500'
        #startTime = '1340309300' # Set battery cost to 40
        #endTime = '1340309450'
        #startTime = '1340533700' # Set battery cost to 80
        #endTime = '1340533850'
        #startTime = '1340633800' # Battery 60
        #endTime = '1340633950'
        #startTime = '1340634200'
        #endTime = '1340634350'
        #startTime = '1340643650'
        #endTime = '1340643900'
        #startTime = '1340644650' #Std dev cost 150 (battery 40)
        #endTime = '1340644750'
        #startTime = '1341075100' # Better path following, etc.
        #endTime = '1341075180'
        #startTime = '1341077900' # Better path following, etc.
        #endTime = '1341078050'
        #startTime = '1341148700' # Simple Ik for exploration
        #endTime = '1341148900'
        #startTime = '1341157900' # Same, 15 sims
        #endTime = '1341158100'
        #startTime = '1341165400' # Reduced ik rotations
        #endTime = '1341165600'
        startTime = '1341343400' # Exp mapped thresh to 0.5
        endTime = '1341343500'
        
        
        expDirs = [x for x in os.listdir(relDir) if 'sim' in x]
        expDirs = sorted(expDirs)
        dirTuples = [(x, x.split('_')[2]) for x in expDirs]
        expDirs = [x[0] for x in dirTuples if x[1] > startTime and x[1] < endTime]
        dirs.extend(expDirs)
    
    resultsFiles = []
    for simDir in dirs:
        absSimDir = os.path.join(relDir, simDir)
        files = os.listdir(absSimDir)
        boardFile = [os.path.join(absSimDir, x) for x in files if x.startswith('board_')][0]
        robotFiles = [os.path.join(absSimDir, x) for x in files if x.startswith('robot_')]

        filesTuple = (boardFile, robotFiles, absSimDir)
        resultsFiles.append(filesTuple)
    return resultsFiles, dirs

def processResults():
    resultsFiles, dirs = gatherSimFiles()
    #resultsFiles, dirs = gatherExperimentFiles()
    
    print('Results files...')
    pprint.pprint(resultsFiles)

    resultsDictList = extractResults.extractResultsFromExperiments(resultsFiles)
    analyseResults.analyseResultsDictList(resultsDictList)
    printResults.printResultsToCsvFiles(resultsDictList, dirs)

def refineRotation():
    #reSearchPose = re.compile('<RobotStatus>.+?pose=\((.+?),(.+?),(.+?)\)').search
    reSearchMove = re.compile('<bCMU_MoveRobot>usBurst=(.+?) dir=(.+?) isHBridge').search
    reSearchCompass = re.compile('<CompassReading>currentOrient=(.+?) reading=(.+?)</CompassReading>').search

    completeTuples = []
    experimentFiles = gatherExperimentFiles()
    for (boardFile, robotFiles, outputDir) in experimentFiles:
        for robotFile in robotFiles:
            moveTuples = []
            compassTuples = []
            lineIndex = 0
            f = open(robotFile, 'r')
            for line in f:
                m1 = reSearchMove(line)
                m2 = reSearchCompass(line)
                if m1:
                    moveTuple = (lineIndex, m1.group(1), m1.group(2))
                    moveTuples.append(moveTuple)
                elif m2:
                    compassTuple = (lineIndex, m2.group(1), m2.group(2))
                    compassTuples.append(compassTuple)
                lineIndex += 1

            compassIndex = 0
            for moveIndex in range(len(moveTuples)):
                # Get compass before and after move
                while True:
                    if compassTuples[compassIndex + 1][0] > moveTuples[moveIndex][0]:
                        break
                    compassIndex += 1
                prevOrient = compassTuples[compassIndex][2]
                nextEstdOrient = compassTuples[compassIndex + 1][1]
                nextOrient = compassTuples[compassIndex + 1][2]
                completeTuple = (moveTuples[moveIndex][1], moveTuples[moveIndex][2], prevOrient, nextEstdOrient, nextOrient, moveTuples[moveIndex][0], compassTuples[compassIndex][0], compassTuples[compassIndex + 1][0])
                completeTuples.append(completeTuple)


    print('left')
    measuredDeltaOrients = []
    for completeTuple in (x for x in completeTuples if x[1] == '2'):
        estdDeltaOrient = geometryUtils.orientDiff(float(completeTuple[2]), float(completeTuple[3]))
        measuredDeltaOrient = geometryUtils.orientDiff(float(completeTuple[2]), float(completeTuple[4]))
        print('estd %12f measured %12f (%4d, %4d-%4d)' % (estdDeltaOrient, measuredDeltaOrient, completeTuple[5], completeTuple[6], completeTuple[7]))
        if measuredDeltaOrient > 0:  # Some measurements/moves were wrong :(
            measuredDeltaOrients.append(measuredDeltaOrient)
    avgDeltaOrient = sum(measuredDeltaOrients) / len(measuredDeltaOrients)
    varDeltaOrient = 0.0
    for measuredDeltaOrient in measuredDeltaOrients:
        varDeltaOrient += math.pow(avgDeltaOrient - measuredDeltaOrient, 2)
    varDeltaOrient /= len(measuredDeltaOrients)
    print('avg measured %12f var %12f (from %d measurements)' % (avgDeltaOrient, varDeltaOrient, len(measuredDeltaOrients)))
    
    print('right')
    measuredDeltaOrients = []
    for completeTuple in (x for x in completeTuples if x[1] == '3'):
        estdDeltaOrient = geometryUtils.orientDiff(float(completeTuple[2]), float(completeTuple[3]))
        measuredDeltaOrient = geometryUtils.orientDiff(float(completeTuple[2]), float(completeTuple[4]))
        print('estd %12f measured %12f' % (estdDeltaOrient, measuredDeltaOrient))
        measuredDeltaOrients.append(measuredDeltaOrient)
    avgDeltaOrient = sum(measuredDeltaOrients) / len(measuredDeltaOrients)
    varDeltaOrient = 0.0
    for measuredDeltaOrient in measuredDeltaOrients:
        varDeltaOrient += math.pow(avgDeltaOrient - measuredDeltaOrient, 2)
    varDeltaOrient /= len(measuredDeltaOrients)
    print('avg measured %12f var %12f (from %d measurements)' % (avgDeltaOrient, varDeltaOrient, len(measuredDeltaOrients)))

    print('fwd')
    fwdTuples = []
    measuredDeltaOrients = []
    avgDiffBetweeenEstdAndMeasured = 0.0
    fwdCompleteTuples = [x for x in completeTuples if x[1] == '0']
    for completeTuple in fwdCompleteTuples:
        estdDeltaOrient = geometryUtils.orientDiff(float(completeTuple[2]), float(completeTuple[3]))
        measuredDeltaOrient = geometryUtils.orientDiff(float(completeTuple[2]), float(completeTuple[4]))
        fwdTuples.append((float(completeTuple[0]), estdDeltaOrient, measuredDeltaOrient))
        avgDiffBetweeenEstdAndMeasured += (measuredDeltaOrient - estdDeltaOrient)
        measuredDeltaOrients.append(measuredDeltaOrient)
    fwdTuples.sort(key=lambda x: x[0])
    for fwdTuple in fwdTuples:
        print('burst %12f estd %12f measured %12f' % fwdTuple)
    avgDeltaOrient = sum(measuredDeltaOrients) / len(measuredDeltaOrients)
    varDeltaOrient = 0.0
    for measuredDeltaOrient in measuredDeltaOrients:
        varDeltaOrient += math.pow(avgDeltaOrient - measuredDeltaOrient, 2)
    varDeltaOrient /= len(measuredDeltaOrients)
    print('avg measured %12f var %12f (from %d measurements)' % (avgDeltaOrient, varDeltaOrient, len(measuredDeltaOrients)))
    print('avg (measured - estd) delta orient %f' % (avgDiffBetweeenEstdAndMeasured / len(fwdCompleteTuples)))
    return

def extractLocalMapsForReplaying():
    print('Extracting local maps for replaying')
    
    robotLog = r'..\Files\20130728_gumstixFiles_2__collabOkButLoopCloseFailed\gum2\experiment_robot_0_194-19691231-Wed-160314\gum_0_202.txt'
    localMaps = extractLocalMaps.getMapsFromLog(robotLog)
    
    outputFilename = '__localMapArray__.c'
    extractLocalMaps.populateStructsForReplaying(localMaps, outputFilename)
    return


"""
Notes on csv files produced:

* closeLoopScans - for each scan processed when closing a loop, print the offset and std dev for original/adjusted
* closeLoopScans2 - print offset and std dev for adjusted only *per session* - useful in order to determine typical range and average std dev when exploring in a coalition
* closeLoopSessions - n scans when starting/performing close loop, avg std dev, etc




"""

if __name__ == '__main__':
    processResults()
    #extractLocalMapsForReplaying()
    #refineRotation()
    

    print('done')