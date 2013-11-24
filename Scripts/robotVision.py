
import argparse
import time
import sys
from pdb import set_trace
from RobotVisionUtils import groupImages
from RobotVisionUtils import groupBackgroundCells
from RobotVisionUtils import groupColours
from RobotVisionUtils import testObstRec
from RobotVisionUtils import testRobotRec
from RobotVisionUtils import robotVisionUtils






if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Train robot vision")
    parser.add_argument('--mode', type=str, choices=['trainor', 'testor', 'testor2', 'trainrr', 'testrr'], required=True, help='either train robot vision, or test model against data')
    args = parser.parse_args(sys.argv[1:])


    startTimeMain = time.clock()

    if args.mode == 'trainor':
        outputFile = open('__trainObstacleRecognitionOutput__.txt', 'w')
        imageGroups, imageDescs, imageStdDevs = groupImages.groupImagesNEW3(robotVisionUtils.OBST_REC_DATA_DIR, outputFile)
        groupImages.checkImageGroups(imageGroups, imageDescs, imageStdDevs, outputFile)
        groupBackgroundCells.groupCells(imageGroups, imageStdDevs, imageDescs, outputFile)
        groupImages.writeImageGroupsNEW(imageGroups, imageStdDevs)
        outputFile.close()
    elif args.mode == 'testor':
        outputFile = open('__testObstacleRecognitionOutput__.txt', 'w')
        imageGroups = robotVisionUtils.loadPickledItem(robotVisionUtils.IMAGE_GROUPS_PICKLE)
        imageStdDevs = robotVisionUtils.loadPickledItem(robotVisionUtils.IMAGE_STDDEVS_PICKLE)
        testObstRec.testObstRecNEW(imageGroups, imageStdDevs, outputFile)
        #testObstRec.testObstRecWithSmoothingNEW(imageGroups, imageStdDevs, outputFile)  # This wasn't benefitial
        outputFile.close()
    elif args.mode == 'testor2':
        outputFile = open('__testObstacleRecognitionOutput2__.txt', 'w')
        imageGroups = robotVisionUtils.loadPickledItem(robotVisionUtils.IMAGE_GROUPS_PICKLE)
        imageStdDevs = robotVisionUtils.loadPickledItem(robotVisionUtils.IMAGE_STDDEVS_PICKLE)
        imageDescs = groupImages.loadImageDescs(robotVisionUtils.OBST_REC_DATA_DIR)
        groupImages.checkImageGroups(imageGroups, imageDescs, imageStdDevs, outputFile)
        outputFile.close()
    elif args.mode == 'trainrr':
        outputFile = open('__trainRobotRecognitionOutput__.txt', 'w')
        colourGroups = groupColours.groupColoursNEW(robotVisionUtils.ROBOT_REC_DATA_DIR, outputFile)
        groupColours.writeColourGroups(colourGroups)
        outputFile.close()
    elif args.mode == 'testrr':
        outputFile = open('__testRobotRecognitionOutput__.txt', 'w')
        colourGroups = robotVisionUtils.loadPickledItem(robotVisionUtils.COLOUR_GROUPS_PICKLE)
        colourAvgs = testRobotRec.testRobotRec(colourGroups, outputFile)
        testRobotRec.testRobotRecUsingAvgs(colourGroups, colourAvgs, outputFile)
        testRobotRec.testRobotRecAgainstBackgroundImages(colourGroups, colourAvgs, outputFile)
        groupColours.writeColourAvgs(colourAvgs)
        groupColours.writeColourGroupsCode(colourGroups, colourAvgs)
        outputFile.close()


    mainTime = time.clock() - startTimeMain
    print('OVERALL TIME %f' % mainTime)