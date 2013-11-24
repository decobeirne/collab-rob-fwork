import utils
import os, re, math, time, pickle, sys
import pprint, pdb

FIND_COMPASS_READINGS = re.compile(r'<CompassReading>currentOrient=(.+?) reading=(.+?)</CompassReading>').finditer

def calcAvgOrient(filePath):
    pdb.set_trace()
    text = open(filePath).read()
    compassIter = FIND_COMPASS_READINGS(text)
    diffList = []
    lastOrient = -1000.0
    for m in compassIter:
        current = float(m.group(1))
        reading = float(m.group(2))
        if current == lastOrient:
            continue
        lastOrient = current
        if reading < 1 and current > 6:
            reading += math.pi * 2
        elif reading > 6 and current < 1:
            current += math.pi * 2
        diff = reading - current
        diffList.append(diff)
    pdb.set_trace()
    absDiffList = [abs(x) for x in diffList]
    avgAbsDiff = sum(absDiffList) / len(absDiffList)
    avgDiff = sum(diffList) / len(diffList)
    print('avg diff %f avg abs diff %f' % (avgDiff, avgAbsDiff))
    print('avg diff %f avg abs diff %f' % (avgDiff * (180 / math.pi), avgAbsDiff * (180 / math.pi)))
    greaterDiffs = [x for x in diffList if x > 0]
    lessDiffs = [x for x in diffList if x <= 0]
    avgGreaterDiff = sum(greaterDiffs) / len(greaterDiffs)
    avgLessDiff = sum(lessDiffs) / len(lessDiffs)
    print('avg %d greater diffs %f' % (len(greaterDiffs), avgGreaterDiff))
    print('avg %d less diffs %f' % (len(lessDiffs), avgLessDiff))


def main():
    """
    Go through log file from gumstix experiment and calculate the difference
    between what we expected the error at a new location to be, and what it
    actually is.
    """
    path0 = r'..\Files\20120806_pcGumstixExperiments\experiment_robot_0_118-19691231-Wed-160158\gum_0_126.txt'
    calcAvgOrient(path0)


if __name__ == '__main__':
    main()
