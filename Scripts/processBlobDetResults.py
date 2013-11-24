import os
import re
import math
import pprint
from pdb import set_trace
import geometryUtils


    
    
    
    
    
def processResults(useCopy=False):
    if useCopy:
        resultsFilename = '__blobDetResultsCopy__.txt'
    else:
        resultsFilename = r'../__RobotRecognitionBlobDetectionTestOutput__.txt'
    resultsText = open(resultsFilename).read()
    if not useCopy:
        open('__blobDetResultsCopy__.txt', 'w').write(resultsText)
    imageIter = re.finditer('<TestRobotRecognition>image=(\S)', resultsText, re.DOTALL)

    # Get text for each image tested
    imageList = []
    lastImage = None
    for imageMatch in imageIter:
        startIndex = imageMatch.start()
        imageNum = imageMatch.group(1)
        thisImage = {'start': startIndex, 'imageNum': int(imageNum)}
        if lastImage:
            lastImage['end'] = startIndex
        imageList.append(thisImage)
        lastImage = thisImage
    thisImage['end'] = len(resultsText)

    def __calcStdDev(cov):
        """
        Calc stdDev from cov - as calcd in Uncertainty_calcEigenvalues.
        Actually, here we pretty stupidly assume cov is aligned to x,y axes
        """
        area = math.sqrt(cov[0]) * math.sqrt(cov[3])
        return area
    
    def __calcError(est, focalPt, actualLoc):
        absEstX = est[0] + focalPt[0]
        absEstY = est[1] + focalPt[1]
        errorX = absEstX - actualLoc[0]
        errorY = absEstY - actualLoc[1]
        errorLen = math.sqrt(errorX * errorX + errorY * errorY)
        return (errorX, errorY, errorLen)

    def __getValsFromResultsMatch(resultsMatch):
        focalPt = [float(x) for x in resultsMatch.group(1).split(',')]
        cov = [float(x) for x in resultsMatch.group(2).split(',')]
        error = [float(x) for x in resultsMatch.group(3).split(',')]
        actualLoc = [float(x) for x in resultsMatch.group(4).split(',')]
        stdDev = __calcStdDev(cov)
        results = {'focalPt': focalPt, 'cov': cov, 'stdDev': stdDev, 'error': error, 'actualLoc': actualLoc}
        return results
    

    for image in imageList:
        imageText = resultsText[image['start']:image['end']]
        
        # Currently(!!!)  there will be 2 ests if an accurate est is made (alongside the initial rough est)
        resultsMatch = re.search('<RobotTrainingEst>.+?focalPt=\((.+?)\).+?cov=\((.+?)\).+?error=\((.+?)\) actualLoc=\((.+?)\)</RobotTrainingEst>', imageText)
        accurateEstSearchStartLoc = -1
        image['est'] = resultsMatch is not None
        if resultsMatch:
            incorrectMatch = re.search('<IncorrectBlob>', imageText)
            image['incorrect'] = incorrectMatch is not None
            if incorrectMatch:
                continue
            image['roughResults'] = __getValsFromResultsMatch(resultsMatch)
            faceIter = re.finditer('<FaceRoughLocation>.+?cornerVisible=(\S) corner=\"(.{2})\".+?est=\((.+?)\) cov=\((.+?)\)', imageText)
            if not faceIter:
                print('NO FACE LOCATION FOR %d' % image['imageNum'])
            image['faceMatches'] = []
            for faceMatch in faceIter:
                accurateEstSearchStartLoc = faceMatch.start()
                face = {'hadCorner':int(faceMatch.group(1)), 'corner':faceMatch.group(2), 'est':[float(x) for x in faceMatch.group(3).split(',')], 'cov':[float(x) for x in faceMatch.group(4).split(',')]}
                face['stdDev'] = __calcStdDev(face['cov'])
                face['error'] = __calcError(face['est'], image['roughResults']['focalPt'], image['roughResults']['actualLoc'])
                image['faceMatches'].append(face)
            roughMatch = re.search('<RobotRoughLocation>est=\((.+?)\) cov=\((.+?)\)', imageText)
            if not roughMatch:
                print('NO ROUGH MATCH FOR %d' % image['imageNum'])
            rough = {'est':[float(x) for x in roughMatch.group(1).split(',')], 'cov':[float(x) for x in roughMatch.group(2).split(',')]}
            rough['stdDev'] = __calcStdDev(rough['cov'])
            rough['error'] = __calcError(rough['est'], image['roughResults']['focalPt'], image['roughResults']['actualLoc'])
            image['rough'] = rough
        if accurateEstSearchStartLoc:
            #set_trace()
            accFaceEstIter = re.finditer('<FaceEdgeAccLoc>colourId=(.+?) faceIndex=(.+?) edgeIndex=(.+?) loc=\((.+?),(.+?)\) cov=\((.+?),(.+?),(.+?),(.+?)\)</FaceEdgeAccLoc>', imageText)
            image['accFaceEdgeEsts'] = []
            for accFaceEstMatch in accFaceEstIter:
                m = accFaceEstMatch
                accFace = {'colId': int(m.group(1)), 'faceInd': int(m.group(2)), 'edgeInd': int(m.group(3)), 'loc': (float(m.group(4)), float(m.group(5))), 'cov': (float(m.group(6)),float(m.group(7)),float(m.group(8)),float(m.group(9)))}
                accFace['error'] = __calcError(accFace['loc'], image['roughResults']['focalPt'], image['roughResults']['actualLoc'])
                image['accFaceEdgeEsts'].append(accFace)
            accFaceLocIter = re.finditer('<FaceAccLoc>colourId=(.+?) faceIndex=(.+?) nEdgeEsts=(.+?) loc=\((.+?),(.+?)\) cov=\((.+?),(.+?),(.+?),(.+?)\)</FaceAccLoc>', imageText)
            image['accFaceEsts'] = []
            for accFaceMatch in accFaceLocIter:
                m = accFaceMatch
                accFace = {'colId': int(m.group(1)), 'faceInd': int(m.group(2)), 'nEdgeEsts': int(m.group(3)), 'loc': (float(m.group(4)), float(m.group(5))), 'cov': (float(m.group(6)),float(m.group(7)),float(m.group(8)),float(m.group(9)))}
                accFace['error'] = __calcError(accFace['loc'], image['roughResults']['focalPt'], image['roughResults']['actualLoc'])
                image['accFaceEsts'].append(accFace)
            if '<RobotAccurateLocation>' in imageText:
                # Find first result for rough est, and skip this
                startLoc = imageText.index('</RobotTrainingEst>')
                if startLoc != -1:
                    resultsMatch = re.search('<RobotTrainingEst>.+?focalPt=\((.+?)\).+?cov=\((.+?)\).+?error=\((.+?)\) actualLoc=\((.+?)\)</RobotTrainingEst>', imageText[startLoc:])
                    image['accEst'] = resultsMatch is not None
                    if resultsMatch:
                        image['accResults'] = __getValsFromResultsMatch(resultsMatch)


    imageList2 = [x for x in imageList if x['est'] and not x['incorrect']]

    faceEstsWithCorner = []
    faceEstsWithoutCorner = []
    allFaces = []
    for image in imageList2:
        for face in image['faceMatches']:
            if face['hadCorner']:
                faceEstsWithCorner.append(face)
            else:
                faceEstsWithoutCorner.append(face)
            allFaces.append(face)
    
    estsWithMultFaces = [x for x in imageList2 if len(x['faceMatches']) > 1]
    estsWithOneFace = [x for x in imageList2 if x not in estsWithMultFaces]

    def __getAvgs(estList, calcVar=1):
        # Remember for testing robots are at orient 0, so global x axis is up/dist in the images
        nEsts = len(estList)
        varX = sum(x['error'][0] * x['error'][0] for x in estList) / nEsts
        varY = sum(x['error'][1] * x['error'][1] for x in estList) / nEsts
        covar = sum(x['error'][0] * x['error'][1] for x in estList) / nEsts
        avgX = sum(math.fabs(x['error'][0]) for x in estList) / nEsts
        avgY = sum(math.fabs(x['error'][1]) for x in estList) / nEsts
        print('avg x %f' % avgX)
        print('avg y %f' % avgY)
        print('var x %f' % varX)
        print('var y %f' % varY)
        print('covar %f' % covar)
        '''avgErrorX = sum(math.fabs(x['error'][0]) for x in estList) / nEsts
        avgErrorY = sum(math.fabs(x['error'][1]) for x in estList) / nEsts
        avgVarX = sum(math.fabs(x['cov'][0]) for x in estList) / nEsts
        avgVarY = sum(math.fabs(x['cov'][3]) for x in estList) / nEsts
        print('n %d' % nEsts)
        print('avg error x %f' % avgErrorX)
        print('avg error y %f' % avgErrorY)
        print('avg var x %f' % avgVarX)
        print('avg var y %f' % avgVarY)
        
        if calcVar:
            varXX = avgErrorX * avgErrorX
            varXY = avgErrorX * avgErrorY
            varYY = avgErrorY * avgErrorY
            print('var x %f' % varXX)
            print('var y %f' % varYY)
            print('covar %f' % varXY)'''
        

    print('Error for faces with corner:')
    __getAvgs(faceEstsWithCorner)

    print('Error for faces without corner:')
    __getAvgs(faceEstsWithoutCorner)
    
    print('Error for all faces:')
    __getAvgs(allFaces, 1)
    
    #set_trace()
    print('Error for all rough ests:')
    __getAvgs([x['roughResults'] for x in imageList2])
    
    print('Error for rough ests with 1 face:')
    __getAvgs([x['roughResults'] for x in estsWithOneFace])
    
    print('Error for rough ests with n faces:')
    __getAvgs([x['roughResults'] for x in estsWithMultFaces])
    
    #set_trace()
    
    accFaceEdges = []
    for x in imageList2:
        if 'accFaceEdgeEsts' in x:
            accFaceEdges.extend(x['accFaceEdgeEsts'])

    vertAccFaceEdges = [x for x in accFaceEdges if x['edgeInd'] == 2 or x['edgeInd'] == 3]
    horAccFaceEdges = [x for x in accFaceEdges if x not in vertAccFaceEdges]
    
    print('Error for hor acc face edges')
    __getAvgs(horAccFaceEdges)
    
    print('Error for vert acc face edges')
    __getAvgs(vertAccFaceEdges)
    
    print('Error for all acc face edges')
    __getAvgs(accFaceEdges)
    
    accFaces = []
    for x in imageList2:
        if 'accFaceEsts' in x:
            accFaces.extend(x['accFaceEsts'])

    '''accResults = []
    for x in imageList2:
        if 'accResults' in x:
            accResults.append(x['accResults'])

    print('Error for acc ests')
    __getAvgs(accResults)'''
    
    print('Avg est dist')
    focalLens = [math.fabs(x['roughResults']['focalPt'][0]) for x in imageList2]
    print('n %d' % len(focalLens))
    print('avg %f' % (sum(focalLens) / len(focalLens)))
    
    #set_trace()
        

        
        
    
    # Get stdDev for face and rough - should be comping this to error to see if we need to inc scale of cov
    # Get error for each face est - det what this should be for corner/noCorner
    # Get error for rough est with 1face/2faces - wait is this even reasonable, we combine the matrices correctly after all? - useful to verify stuff anyway
    
    pass


def checkHoughLines():
    pass



if __name__ == '__main__':
    processResults()
    checkHoughLines()
    
    print('done')