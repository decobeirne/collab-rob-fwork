"""
Take a directory containing board thread*.txt files and [robot*.txt|gum*.txt]
files, and copy output from cloud processes run in thread files into
the corresponding robot/gum files.

The robot/gum files should have been copied into the board's log dir. Each thread
file has the name of the appropriate robot file.

This functionality is useful/necessary when comparing recorded files to rerun files.
"""
import sys
import os
import re
from pdb import set_trace

def updateRobotLog():
    #d0 = r'..\Files\experiment_board_1367139764-20130428-Sun-100244__cloud'
    #d0 = r'..\Files\experiment_board_1367144806-20130428-Sun-112646__gum'
    #d0 = r'..\Files\experiment_board_1367165799-20130428-Sun-171639__gum'
    #d0= r'..\Files\experiment_board_1367171392-20130428-Sun-184952__gum'
    #d0 = r'..\Files\experiment_board_1367171686-20130428-Sun-185446__gum'
    #d0 = r'D:\thesis\code\CollabExp_b\Files\experiment_board_1368947753-20130519-Sun-081553'
    #d0 = r'D:\thesis\code\CollabExp\Files\20130519_gumstixFiles_2__indepAndCollabExpers\experiment_board_1368974024-20130519-Sun-153344'
    #d0 = r'D:\thesis\code\CollabExp\Files\20130519_gumstixFiles_2__indepAndCollabExpers\experiment_board_1368974396-20130519-Sun-153956'
    d0 = r'D:\thesis\code\CollabExp\Files\20130519_gumstixFiles_2__indepAndCollabExpers\experiment_board_1368975061-20130519-Sun-155101'

    dirContainingThreadAndRobotFile = d0
    
    
    filesInDir = os.listdir(dirContainingThreadAndRobotFile)
    matchingFiles = []
    def __getFilesForRobotIndex(filePattern):
        return [x for x in filesInDir if x.startswith(filePattern % i)]
    for i in range(100):
        #threadFiles = [x for x in filesInDir if x.startswith('thread_%d' % i)]
        threadFiles = __getFilesForRobotIndex('thread_%d')
        robotFiles = __getFilesForRobotIndex('robot_%d')
        gumFiles = __getFilesForRobotIndex('gum_%d')
        if not threadFiles or not (robotFiles or gumFiles):
            print('Couldn\'t match files for index %d' % i)
            break
        if robotFiles:
            matchingFiles.append((threadFiles[0], robotFiles[0]))
        else:
            matchingFiles.append((threadFiles[0], gumFiles[0]))
    for threadFileName, robotFileName in matchingFiles:
        print('Processing %s and %s' % (threadFileName, robotFileName))
        threadText = open(os.path.join(dirContainingThreadAndRobotFile, threadFileName)).read()
        robotText = open(os.path.join(dirContainingThreadAndRobotFile, robotFileName)).read()
        
        cloudExpIter = re.finditer('<ReadExpJob>.+?</PayloadEXPLORATION>(.+?)</ReadExpJob>', threadText, re.DOTALL)
        cloudGtepIter = re.finditer('<ReadGtepJob>.+?</PayloadGOTO_EXP_PT>(.+?)</ReadGtepJob>', threadText, re.DOTALL)
        robotExpIter = re.finditer('<CloudExploration />', robotText)
        robotGtepIter = re.finditer('<CloudGotoExpPt />', robotText)

        cloudExpList = []
        for m in cloudExpIter:
            cloudExpList.append(m.group(1))
        cloudGtepList = []
        for m in cloudGtepIter:
            cloudGtepList.append(m.group(1))
        
        cloudPlaceholder = '<CloudExploration />'
        gtepPlaceholder = '<CloudGotoExpPt />'
        
        if 1:
            index = 0
            while True:
                pos = robotText.find(cloudPlaceholder)
                if pos == -1:
                    break
                if index >= len(cloudExpList):
                    print('WARNING: cannot find board section (only %d) for cloud section %d' % (len(cloudExpList), index))
                robotText = robotText.replace(cloudPlaceholder, cloudExpList[index], 1)
                index += 1
                print('replaced exp')

        if 1:
            index = 0
            while True:
                pos = robotText.find(gtepPlaceholder)
                if pos == -1:
                    break
                robotText = robotText.replace(gtepPlaceholder, cloudGtepList[index], 1)
                index += 1
                print('replaced gtep')

        outputRobotFileName = robotFileName.replace('.txt', '__updated.txt')
        outputRobotFile = open(os.path.join(dirContainingThreadAndRobotFile, outputRobotFileName), 'w')
        outputRobotFile.write(robotText)
        outputRobotFile.close()

    return 0



if __name__ == '__main__':
    sys.exit(updateRobotLog())
