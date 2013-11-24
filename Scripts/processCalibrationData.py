import pprint
import math
from pdb import set_trace
import calibrationData
import geometryUtils

Vec3 = geometryUtils.Vec3
Mat3 = geometryUtils.Mat3

# Verbose flags
loud = True
quiet = False


################################################################################################
#
# Camera Calibration
#
################################################################################################


def __estCamTheta(distFromP, height, pixelVal, camPixelMidpt, focalLen, camHeight, verbose):
    pixelOffset = math.fabs(pixelVal - camPixelMidpt)

    # The focal len is stated in terms of image pixels. Therefore when a pt is projected onto the
    # image plane (i.e. through the lens), then the angle of the ray to that pt in space can be calculated
    # relative to the camera
    angleDiff = math.atan(pixelOffset / focalLen)

    angleInWorld = math.atan((camHeight - height) / distFromP)

    # I have adjusted the training measurements such that the origin is at the bottom-left of the image,
    # thus the same as when we run the code
    isFurther = (pixelVal > camPixelMidpt)
    if isFurther:
        camThetaEst = angleInWorld + angleDiff
    else:
        camThetaEst = angleInWorld - angleDiff

    if verbose == loud:
        print('pixel %3d theta in world: %.15f theta diff from centre: %.15f (%d) centre theta est: %.15f' % (pixelVal, angleInWorld, angleDiff, isFurther, camThetaEst))

    return camThetaEst


def calcCamThetaNEW2(measurements):
    '''
    Calculate camera theta relative to horizontal plane. Assume the robot is on a flat plane. Also assume
    that the camera is pointing exactly forward relative to a vertical axis through the robot's centre.

    Measurements are given in terms of the distance on the ground from the camera's P, and the corresponding
    pixel in the image. Given the mid point of the image, and the focal length of the camera, the angle between
    a ray cast to the point in space and the camera's orientation can be calculated. Given this angle, and the
    angle to the actual point in space, and estimate of the camera's orientation can be made.
    '''
    def __decimalRange(start, stop, step):
        val = start
        while val < stop:
            yield val
            val += step

    midPixel = measurements.camMidptPixel[1]
    midPixelRange = 3
    midPixelStep = 1
    camHeight = measurements.camHeight
    camHeightRange = 30
    camHeightStep = 1
    camTheta = measurements.camTheta
    camThetaRange = 0.5
    camThetaStep = 0.01
    distAdjustment = 0
    distAdjustmentRange = 30
    distAdjustmentStep = 1

    fabsFunc = math.fabs
    atanFunc = math.atan
    tanFunc = math.tan
    print('initial vals are: %d %f %f ' % (midPixel, camHeight, camTheta))

    bestValues = None
    bestFocalLens = None
    bestError = 1000000
    tempList = []

    # Try and calc focal pt from fairly good initial estimates for other values
    #for mp in __decimalRange(midPixel - midPixelRange, midPixel + midPixelRange, midPixelStep):
    if 1:
        #mpd = mp #* 10
        mpd = midPixel
        for ch in __decimalRange(camHeight - camHeightRange, camHeight + camHeightRange, camHeightStep):
            for ct in __decimalRange(camTheta - camThetaRange, camTheta + camThetaRange, camThetaStep):
                for da in __decimalRange(distAdjustment - distAdjustmentRange, distAdjustment + distAdjustmentRange, distAdjustmentStep):
                    focalLens = []
                    for pixelDist, height, pixelVal in measurements.camThetaPixelVals:
                        ratio = (ch - height) / (pixelDist + da)
                        angle = atanFunc(ratio)
                        angleDiff = fabsFunc(angle - ct)

                        pvd = pixelVal #* 10
                        pixelDiff = fabsFunc(pvd - mpd)

                        focalSpaceRatio = tanFunc(angleDiff)

                        # ratio = opp/adj
                        # ratio * adj = opp
                        # adj = opp / ratio
                        fcd = pixelDiff / focalSpaceRatio
                        focalLens.append(fcd)
                    avg = sum(focalLens) / len(focalLens)
                    stdDev = sum(fabsFunc(x - avg) for x in focalLens)
                    #if stdDev < bestError:
                    #    bestError = stdDev
                    #    bestFocalLens = tuple(focalLens)
                    #    bestValues = (avg, stdDev, mp, ch, ct, da)
                    anyInput = (mpd, ch, ct, da)
                    anyFocalLens = tuple(focalLens)
                    anyValues = (anyInput, anyFocalLens, avg, stdDev, stdDev / avg)
                    tempList.append(anyValues)

    if 1:  # debugging:
        #f0 = open("__focalLenVals__.txt", "w")
        import pdb; pdb.set_trace()

    if bestValues:
        focalLen = bestValues[-6]
        midpixel = bestValues[-4]
        camHeight = bestValues[-3]
        theta = bestValues[-2]
        adjustment = bestValues[-1]

        newPDist = measurements.pDist - adjustment

        measurements.focalLen = focalLen
        measurements.camHeight = camHeight
        measurements.camtTheta = theta
        measurements.pDist = newPDist
        measurements.camMidptPixel[1] = midpixel

        print('best values:')
        pprint.pprint(bestValues)

        print('from focal lens:')
        pprint.pprint(bestFocalLens)

        return measurements
    else:
        return measurements


def calcCamThetaNEW(measurements):
    '''
    Calculate camera theta relative to horizontal plane. Assume the robot is on a flat plane. Also assume
    that the camera is pointing exactly forward relative to a vertical axis through the robot's centre.

    Measurements are given in terms of the distance on the ground from the camera's P, and the corresponding
    pixel in the image. Given the mid point of the image, and the focal length of the camera, the angle between
    a ray cast to the point in space and the camera's orientation can be calculated. Given this angle, and the
    angle to the actual point in space, and estimate of the camera's orientation can be made.
    '''
    camThetaEsts = []

    def __decimalRange(start, stop, step):
        val = start
        while val < stop:
            yield val
            val += step

    midPixel = measurements.camMidptPixel[1]
    midPixelRange = 3
    midPixelStep = 0.5
    camHeight = measurements.camHeight
    camHeightRange = 30
    camHeightStep = 1
    camTheta = 0.5
    camThetaRange = 0.4
    camThetaStep = 0.005
    distAdjustment = 0
    distAdjustmentRange = 30
    distAdjustmentStep = 1

    fabsFunc = math.fabs
    atanFunc = math.atan
    tanFunc = math.tan
    print('initial vals are: %d %f %f ' % (midPixel, camHeight, camTheta))


    #set_trace()
    bestValues = None
    bestError = 1000000
    # Try and calc focal pt from fairly good initial estimates for other values
    for mp in __decimalRange(midPixel - midPixelRange, midPixel + midPixelRange, midPixelStep):
        mpd = mp * 10
        for ch in __decimalRange(camHeight - camHeightRange, camHeight + camHeightRange, camHeightStep):
            for ct in __decimalRange(camTheta - camThetaRange, camTheta + camThetaRange, camThetaStep):
                for da in __decimalRange(distAdjustment - distAdjustmentRange, distAdjustment + distAdjustmentRange, distAdjustmentStep):


                    l0 = []
                    for pDist, height, pixelVal in measurements.camThetaPixelVals:
                        ratio = (ch - height) / (pDist + da)
                        angle = atanFunc(ratio)
                        angleDiff = fabsFunc(angle - ct)

                        pvd = pixelVal * 10
                        pixelDiff = fabsFunc(pvd - mpd)

                        focalSpaceRatio = tanFunc(angleDiff)
                        # ratio = opp/adj
                        # ratio * adj = opp
                        # adj = opp / ratio
                        fcd = pixelDiff / focalSpaceRatio
                        l0.append(fcd)
                    avg = sum(l0) / len(l0)
                    stdDev = sum(fabsFunc(x - avg) for x in l0)
                    '''l0.append(avg)
                    l0.append(stdDev)
                    l0.append(mp)
                    l0.append(ch)
                    l0.append(ct)
                    l0.append(da)
                    t0 = tuple(l0)'''
                    #focalLens.append(t0)
                    if stdDev < bestError:
                        bestError = stdDev
                        '''l0.append(avg)
                        l0.append(stdDev)
                        l0.append(mp)
                        l0.append(ch)
                        l0.append(ct)
                        l0.append(da)
                        t0 = (avg, stdDev, mp, ch, ct, da)'''
                        bestValues = (avg, stdDev, mp, ch, ct, da)


    #errors = [x[-5] for x in focalLens]
    #bestIndex = errors.index(min(errors))
    #bestValues = focalLens[bestIndex]

    focalLen = bestValues[-6]
    midpixel = bestValues[-4]
    camHeight = bestValues[-3]
    theta = bestValues[-2]
    adjustment = bestValues[-1]

    newPDist = measurements.pDist - adjustment

    measurements.focalLen = focalLen
    measurements.camHeight = camHeight
    measurements.camtTheta = theta
    measurements.pDist = newPDist
    measurements.camMidptPixel[1] = midpixel

    print('best values:')
    pprint.pprint(bestValues)

    return measurements


def calcCamTheta(measurements):
    '''
    Calculate camera theta relative to horizontal plane. Assume the robot is on a flat plane. Also assume
    that the camera is pointing exactly forward relative to a vertical axis through the robot's centre.

    Measurements are given in terms of the distance on the ground from the camera's P, and the corresponding
    pixel in the image. Given the mid point of the image, and the focal length of the camera, the angle between
    a ray cast to the point in space and the camera's orientation can be calculated. Given this angle, and the
    angle to the actual point in space, and estimate of the camera's orientation can be made.
    '''
    camThetaEsts = []
    for pDist, height, pixelVal in measurements.camThetaPixelVals:
        camThetaEst = __estCamTheta(pDist, height, pixelVal, measurements.camMidptPixel[1], measurements.focalLen, measurements.camHeight, loud)
        camThetaEsts.append(camThetaEst)

    print('Cam theta estimates:')
    print(camThetaEsts)
    avgEst = sum(camThetaEsts)/len(camThetaEsts)
    print('Avg:%.15f' % avgEst)
    stdDev = sum([math.fabs(x - avgEst) for x in camThetaEsts])/len(camThetaEsts)
    print('Std dev:%.15f' % stdDev)
    return avgEst


def __calcDistError(focalLen, camHeight, camTheta, distAdjustment, distFromP, height, pixelVal, camPixelMidpt, verbose):
    '''
    Determine distFromP from other info and compare to observed value.
    '''
    pixelDiff = math.fabs(pixelVal - camPixelMidpt)
    angleDiff = math.atan(pixelDiff / focalLen)
    isFurther = (pixelVal < camPixelMidpt)
    if isFurther:
        angleInWorld = camTheta - angleDiff
    else:
        angleInWorld = camTheta + angleDiff

    # tan = height/dist
    # dist*tan = height
    # dist = height/tan
    heightOverDist = math.tan(angleInWorld)
    distEst = (camHeight - height) / heightOverDist

    actualDist = distFromP + distAdjustment
    error = math.fabs(distEst - actualDist)

    if verbose:
        print('pixel %d distEst %.15f distFromP %.15f errorNorm %.15f' % (pixelVal, distEst, distFromP, error))
    return error


def refineCamParams(measurements, verbose):
    '''
    3 main variables are: camera theta, camera P height, focal length. Loop over all 3 of these
    to determine if tweaking slightly will make the observed point distances match up better
    '''
    def __decimalRange(start, stop, step):
        val = start
        while val < stop:
            yield val
            val += step

    # Make local copies of values. Adjusted values are printed out and can be used to update measurements.
    focalLen = measurements.focalLen
    camHeight = measurements.camHeight
    camTheta = measurements.camTheta
    camThetaPixelVals = measurements.camThetaPixelVals
    camPixelMidpt = measurements.camMidptPixel[1] # (x,y) tuple. Only want y value.

    minError = 10000
    minValues = {}

    print('Checking distance measurements against camera params. Should take less than 1 minute...')

    focalLenRange = 600
    focalLenStep = 4
    camHeightRange = 20
    camHeightStep = 2
    camThetaRange = .4
    camThetaStep = .01
    adjRange = 30
    adjStep = 2
    camPixelMidptRange = 8
    camPixelMidptStep = 1

    focalLenErrors = {}
    camHeightErrors = {}
    camThetaErrors = {}
    adjErrors = {}


    #for fl in __decimalRange(focalLen - focalLenRange, focalLen + focalLenRange, focalLenStep):
    if 1:
        for ch in __decimalRange(camHeight - camHeightRange, camHeight + camHeightRange, camHeightStep):
            for ct in __decimalRange(camTheta - camThetaRange, camTheta + camThetaRange, camThetaStep):
                for adj in __decimalRange(-adjRange, adjRange, adjStep):
                    #for mp in __decimalRange(camPixelMidpt - camPixelMidptRange ,camPixelMidpt + camPixelMidptRange, camPixelMidptStep):
                    #if 1:
                    for fl in __decimalRange(focalLen - focalLenRange, focalLen + focalLenRange, focalLenStep):
                        mp = camPixelMidpt

                        avgError = 0
                        for distFromP, height, pixelVal in camThetaPixelVals:
                            error = __calcDistError(fl, ch, ct, adj, distFromP, height, pixelVal, mp, verbose)
                            avgError += math.fabs(error)
                        avgError /= len(camThetaPixelVals)
                        if verbose:
                            print('%12f %12f %12f %12f avg error %12f' % (fl, ch, ct, adj, avgError))

                        if avgError < minError:
                            minValues['focalLen'] = fl
                            minValues['camHeight'] = ch
                            minValues['camTheta'] = ct
                            minValues['pDist'] = measurements.pDist - adj  # Adjustement is added to (p-pt) => pDist is dist of (cog-p), so we sub adjustment from this
                            minValues['camMidptPixelY'] = mp
                            #minValues.(fl, ch, ct, adj, mp)
                            minError = avgError

                        if fl not in focalLenErrors:
                            focalLenErrors[fl] = avgError
                        else:
                            focalLenErrors[fl] = min(focalLenErrors[fl], avgError)
                        if ch not in camHeightErrors:
                            camHeightErrors[ch] = avgError
                        else:
                            camHeightErrors[ch] = min(camHeightErrors[ch], avgError)
                        if ct not in camThetaErrors:
                            camThetaErrors[ct] = avgError
                        else:
                            camThetaErrors[ct] = min(camThetaErrors[ct], avgError)
                        if adj not in adjErrors:
                            adjErrors[adj] = avgError
                        else:
                            adjErrors[adj] = min(adjErrors[adj], avgError)

    if minValues:
        print('Min error: %.15f focal %.15f height %.15f theta %.15f pDist %.15f midpt %.15f' % (
            minError,
            minValues['focalLen'],
            minValues['camHeight'],
            minValues['camTheta'],
            minValues['pDist'],
            minValues['camMidptPixelY']))

    print('focalLen errors:')
    pprint.pprint(focalLenErrors)
    print('camHeight errors:')
    pprint.pprint(camHeightErrors)
    print('camTheta errors:')
    pprint.pprint(camThetaErrors)
    print('adj errors:')
    pprint.pprint(adjErrors)

    currentError = 0
    for distFromP, height, pixelVal in camThetaPixelVals:
        error = __calcDistError(focalLen, camHeight, camTheta, 0, distFromP, height, pixelVal, camPixelMidpt, verbose)
        currentError += math.fabs(error)
    currentError /= len(camThetaPixelVals)
    print('Current error: %.15f focal %.15f height %.15f theta %.15f adjustment %.15f midpt %.15f' % (currentError, focalLen, camHeight, camTheta, 0, camPixelMidpt))

    if minError < currentError:
        print('Min error %f < current error %f => updating measurements before printing template' % (minError, currentError))
        measurements.focalLen = minValues['focalLen']
        measurements.camHeight = minValues['camHeight']
        measurements.camtTheta = minValues['camTheta']
        measurements.pDist = minValues['pDist']
        measurements.camMidptPixel[1] = minValues['camMidptPixelY']
    return


camParamsTemplate = '''
%(flipv)s    #define IMG_FLIP_V //!< Should grabbed image be flipped vertically.
%(fliph)s    #define IMG_FLIP_H //!< Should grabbed images be flipped horizontally.
%(flipc)s    #define FLIP_COMPASS //!< Should compass reading be inverted.
%(ishbr)s    #define IS_H_BRIDGE //!< Is robot controlled via h-bridge.
%(ishmc)s    #define IS_HMC6352 //!< Flags if this robot is equipped with a HMC6352 compass module.
%(smtrs)s    #define SWITCH_MOTORS //!< Should commands to differential motors be flipped.

#ifdef IS_HMC6352
#define COMPASS_STDEV                           HMC6352_STD_DEV
#else
#define COMPASS_STDEV                           CMPS03
#endif
#define CAM_P_OFFSET_WORLD                      %(pDist).15ff //!< Offset to cameras focal pt (from centre of robot).
#define CAM_HEIGHT_WORLD                        %(camHeightWorld).15ff //!< Height of camera focal pt.
#define CAM_THETA                               %(camTheta).15ff //!< Angle between camera normal and horizon.
#define FOCAL_LEN                               %(focalLen).15ff //!< Focal length of camera.
#define IMAGE_CENTRE_PIXEL_Y                    %(camMidptPixelY)d //!< Y value in captured images corresponding to centre.
#define CAM_OCCUPANCY_GRID_ORIGIN_X             %(origx)d //!< Coordinate in grabbed images which should be considered the origin.
#define CAM_OCCUPANCY_GRID_ORIGIN_Y             %(origy)d
#define CAM_OCCUPANCY_GRID_Y_EXP                %(camOccupGridy)d //!< Dimensions of grid used for detecting obstacles.
'''


def printCamParams(measurements):
    settingsDict = {}

    # Boolean defines are used with '#ifdef', so comment out to signify 0/1
    settingsDict['flipv'] = ('//', '')[measurements.flipv]
    settingsDict['fliph'] = ('//', '')[measurements.fliph]
    settingsDict['flipc'] = ('//', '')[measurements.flipCompass]
    settingsDict['ishbr'] = ('//', '')[measurements.hBridge]
    settingsDict['ishmc'] = ('//', '')[measurements.hmc6352]
    settingsDict['smtrs'] = ('//', '')[measurements.switchMotors]

    settingsDict['pDist'] = measurements.pDist
    settingsDict['camHeightWorld'] = measurements.camHeight
    settingsDict['camTheta'] = measurements.camTheta
    settingsDict['focalLen'] = measurements.focalLen
    settingsDict['camMidptPixelY'] = measurements.camMidptPixel[1]
    settingsDict['origx'] = measurements.origx
    settingsDict['origy'] = measurements.origy
    settingsDict['camOccupGridy'] = measurements.camOccupGridy
    camParams = camParamsTemplate % settingsDict
    outputFile = open('__camParams__.txt', 'w')
    outputFile.write(camParams)
    outputFile.close()
    print('Params written to __camParams__.txt')



################################################################################################
#
# Movement Calibration
#
################################################################################################

class MoveCalibrationPose:
    def __init__(self, centrePt, fwdVec, fwdOrient, axisLen):
        self.centrePt = centrePt
        self.fwdVec = fwdVec
        self.fwdOrient = fwdOrient
        self.axisLen = axisLen

    def __repr__(self):
        return 'centrePt:%s fwdVec:%s fwdOrient:%f axisLen:%f' % (repr(self.centrePt), repr(self.fwdVec), self.fwdOrient, self.axisLen)

class MoveCalibrationDelta:
    def __init__(self, deltaOrient, fwdMove, latMove):
        self.deltaOrient = deltaOrient
        self.fwdMove = fwdMove
        self.latMove = latMove

    def __repr__(self):
        return 'deltaOrient:%f fwdMove:%f latMove:%f' % (self.deltaOrient, self.fwdMove, self.latMove)

class MoveCalibrationError:
    def __init__(self, varOrient, varFwd, varLat, covar):
        self.varOrient = varOrient
        self.varFwd = varFwd
        self.varLat = varLat
        self.covar = covar

    '''def __init__(self, devOrient, devFwd, devLat):
        self.devOrient = devOrient
        self.devFwd = devFwd
        self.devLat = devLat'''

    def __repr__(self):
        return 'varOrient:%f varFwd:%f varLat:%f covar:%f' % (self.varOrient, self.varFwd, self.varLat, self.covar)

    '''def __repr__(self):
        return 'devOrient:%f devFwd:%f devLat:%f' % (self.devOrient, self.devFwd, self.devLat)'''


class MoveCalibrationResults:
    def __init(self):
        self.avgDeltaOrient = 0
        self.avgDeltaFwd = 0
        self.avgDeltaLat = 0
        self.varOrient = 0
        self.varFwd = 0
        self.varLat = 0
        self.covar = 0

    @classmethod
    def fromVals(cls, avgDeltaOrient, avgDeltaFwd, avgDeltaLat, varOrient, varFwd, varLat, covar):
        res = cls()
        res.avgDeltaOrient = avgDeltaOrient
        res.avgDeltaFwd = avgDeltaFwd
        res.avgDeltaLat = avgDeltaLat
        res.varOrient = varOrient
        res.varFwd = varFwd
        res.varLat = varLat
        res.covar = covar
        return res

    def __repr__(self):
        return 'avgDeltaOrient:%.15f\navgDeltaFwd:%.15f\navgDeltaLat:%.15f\nvarOrient:%.15f\nvarFwd:%.15f\nvarLat:%.15f\ncovar:%.15f' % (
            self.avgDeltaOrient, self.avgDeltaFwd, self.avgDeltaLat, self.varOrient, self.varFwd, self.varLat, self.covar)
        # print('avgDeltaFwd:%.15f avgDeltaLat:%.15f varOrient:%.15f varFwd:%.15f varLat:%.15f covar:%.15f' % self.avgDeltaOrient)
        # print('avgDeltaLat:%.15f varOrient:%.15f varFwd:%.15f varLat:%.15f covar:%.15f' % self.avgDeltaFwd)
        # print('varOrient:%.15f varFwd:%.15f varLat:%.15f covar:%.15f' % self.avgDeltaLat)
        # print('varFwd:%.15f varLat:%.15f covar:%.15f' % self.varOrient)
        # print('varLat:%.15f covar:%.15f' % self.varFwd)
        # print('covar:%.15f' % self.varLat)
        # print('%d:%.15f' % self.covar)


def calcMoveDeltas(locs, areMovesAccumulated, verbose):
    '''
    Return list of offsets per move. If areMovesAccumulated, calculate offset from previous
    loc, otherwise calculate offset from first pose.

    ptDistsToCog: distance from left(front) point and right(back) point to robot's COG
    '''
    # Calculate COG offset from left point as fraction of total dist between measured points.
    cogOffsetFromLeftPt = measurements.leftPtDist / (measurements.leftPtDist + measurements.rightPtDist)

    # Calculate matrix to convert from orientation of vector between measured points to robot's forward vector.
    mat = geometryUtils.calcAxisRotationMat(locs[0])
    poses = []
    for (leftWheel, rightWheel) in locs:
        centrex = leftWheel[0] + cogOffsetFromLeftPt * (rightWheel[0] - leftWheel[0])
        centrey = leftWheel[1] + cogOffsetFromLeftPt * (rightWheel[1] - leftWheel[1])
        diagx = leftWheel[0] - rightWheel[0]
        diagy = leftWheel[1] - rightWheel[1]
        diagVec = (diagx, diagy, 0)
        diagVec = norm(diagVec)
        fwdVec = rotVec3D(diagVec, mat)
        fwdOrient = fwdVec.getOrient()

        pose = (centrex, centrey, fwdOrient, fwdVec)
        poses.append(pose)

    if verbose:
        print('poses:')
        pprint.pprint(poses)

    deltas = []
    for i in range(1,len(poses)):
        if areMovesAccumulated:
            prevPose = poses[i - 1]
        else:
            prevPose = poses[0]
        thisPose = poses[i]
        deltaOrient = orientDiff_rads(prevPose[2], thisPose[2])

        deltaX = thisPose[0] - prevPose[0]
        deltaY = thisPose[1] - prevPose[1]
        offset = (deltaX, deltaY, 0.0)
        fwdMove = dotProd(prevPose[3], offset)
        latMoveVec = crossProd(prevPose[3], offset)
        latMove = vectorLen(latMoveVec)
        if latMoveVec[2] > 0:
            latMove = -latMove

        delta = (deltaOrient, fwdMove, latMove)
        deltas.append(delta)

    if verbose:
        print('deltas:')
        pprint.pprint(deltas)

    return deltas


def calcMoveDeltasNEW(locs, areMovesAccumulated, robotRotMat, centreDistRatioFromLeftPt, verbose):
    '''
    Return list of offsets per move. If areMovesAccumulated, calculate offset from previous
    loc, otherwise calculate offset from first pose.
    '''
    poses = []
    for (leftWheel, rightWheel) in locs:
        leftPt = Vec3(*leftWheel)
        rightPt = Vec3(*rightWheel)
        leftPtToRight = Vec3.sub(rightPt, leftPt)
        centrePt = Vec3(leftPt.x + centreDistRatioFromLeftPt * leftPtToRight.x, leftPt.y + centreDistRatioFromLeftPt * leftPtToRight.y)

        normVec = leftPtToRight.getNorm()
        fwdVec = robotRotMat.rotVec(normVec)
        fwdOrient = fwdVec.getOrient()

        pose = MoveCalibrationPose(centrePt, fwdVec, fwdOrient, leftPtToRight.getLen())
        poses.append(pose)

    if verbose:
        print('poses:')
        pprint.pprint(poses)

    deltas = []

    for i in range(1,len(poses)):
        if areMovesAccumulated:
            prevPose = poses[i - 1]
        else:
            prevPose = poses[0]
        thisPose = poses[i]
        deltaOrient = geometryUtils.orientDiff_rads(prevPose.fwdOrient, thisPose.fwdOrient)
        deltaCentrePt = Vec3.sub(thisPose.centrePt, prevPose.centrePt)
        fwdMove = Vec3.dot(prevPose.fwdVec, deltaCentrePt)
        latMoveVec = Vec3.cross(prevPose.fwdVec, deltaCentrePt)  # Wrong direction but right length, so ok
        latMove = latMoveVec.getLen()
        if latMoveVec.y > 0:
            latMove = -latMove

        delta = MoveCalibrationDelta(deltaOrient, fwdMove, latMove)
        deltas.append(delta)

    if verbose:
        print('deltas:')
        pprint.pprint(deltas)

    return deltas


def calcMoveError(deltas, verbose=quiet):
    '''
    Given set of changes in pose, calculate the average change in orientation, x and y coordinates.
    Also get the variance and covariance in these values.

    Recall that variance is the average of the squared distances from the mean.
    '''
    if verbose == loud:
        pprint.pprint(deltas)

    num = len(deltas)
    avgOrient = sum([x[0] for x in deltas]) / num
    avgX = sum([x[1] for x in deltas]) / num
    avgY = sum([x[2] for x in deltas]) / num
    print('avg delta orient %.15f, delta x %.15f, delta y %.15f' %(avgOrient, avgX, avgY))

    vars = []
    for delta in deltas:
        errorOrient = delta[0] - avgOrient
        errorX = delta[1] - avgX
        errorY = delta[2] - avgY
        varOrient = math.pow(errorOrient, 2)
        varX = math.pow(errorX, 2)
        varY = math.pow(errorY, 2)
        varXY = errorX * errorY
        varTuple = (varOrient, varX, varY, varXY)
        vars.append(varTuple)

    varOrient = sum(x[0] for x in vars) / num
    varX = sum(x[1] for x in vars) / num
    varY = sum(x[2] for x in vars) / num
    varXY = sum(x[3] for x in vars) / num

    print('var delta orient %.15f, delta x %.15f, delta y %.15f' %(varOrient, varX, varY))

    results = (avgOrient, avgX, avgY, varOrient, varX, varY, varXY)
    return results


def calcMoveErrorNEW(deltas, verbose=quiet):
    '''
    Given set of changes in pose, calculate the average change in orientation, x and y coordinates.
    Also get the variance and covariance in these values.

    Recall that variance is the average of the squared distances from the mean.
    '''
    num = len(deltas)
    avgDeltaOrient = sum([x.deltaOrient for x in deltas]) / num
    avgDeltaFwd = sum([x.fwdMove for x in deltas]) / num
    avgDeltaLat = sum([x.latMove for x in deltas]) / num
    print('avg delta orient %.15f, delta x %.15f, delta y %.15f' %(avgDeltaOrient, avgDeltaFwd, avgDeltaLat))

    moveErrors = []
    avgVarOrient = 0
    avgVarFwd = 0
    avgVarLat = 0
    avgCovar = 0
    errorOrients = []
    errorFwds = []
    errorLats = []
    covars = []
    for delta in deltas:
        errorOrient = delta.deltaOrient - avgDeltaOrient
        errorFwd = delta.fwdMove - avgDeltaFwd
        errorLat = delta.latMove - avgDeltaLat
        varOrient = math.pow(errorOrient, 2)
        varFwd = math.pow(errorFwd, 2)
        varLat = math.pow(errorLat, 2)
        covar = errorFwd * errorLat
        
        errorOrients.append(varOrient)
        errorFwds.append(varFwd)
        errorLats.append(varLat)
        covars.append(covar)

        moveError = MoveCalibrationError(varOrient, varFwd, varLat, covar)
        #moveError = MoveCalibrationError(errorOrient, errorFwd, errorLat)
        moveErrors.append(moveError)

    # Get average var diff in each dir
    avgVarOrient = sum(errorOrients) / len(errorOrients)
    avgVarFwd = sum(errorFwds) / len(errorFwds)
    avgVarLat = sum(errorLats) / len(errorLats)
    avgCovar = sum(covars) / len(covars)
    errorVarOrient = sum(abs(x - avgVarOrient) for x in errorOrients) / len(errorOrients)
    errorVarFwd = sum(abs(x - avgVarFwd) for x in errorFwds) / len(errorFwds)
    errorVarLat = sum(abs(x - avgVarLat) for x in errorLats) / len(errorLats)
    errorCovar = sum(abs(x - avgCovar) for x in covars) / len(covars)
    
    #import pdb; pdb.set_trace()

    if verbose == loud:
        print('Errors:')
        pprint.pprint(moveErrors)

    '''stdDevOrient = sum(x.devOrient for x in moveErrors) / num
    stdDevFwd = sum(x.devFwd for x in moveErrors) / num
    stdDevLat = sum(x.devLat for x in moveErrors) / num
    varOrient = stdDevOrient * stdDevOrient
    varFwd = stdDevFwd * stdDevFwd
    varLat = stdDevLat * stdDevLat
    covar = stdDevFwd * stdDevLat'''
    varOrient = sum(x.varOrient for x in moveErrors) / num
    varFwd = sum(x.varFwd for x in moveErrors) / num
    varLat = sum(x.varLat for x in moveErrors) / num
    covar = sum(x.covar for x in moveErrors) / num


    print('var delta orient %.15f, delta x %.15f, delta y %.15f covar %.15f' % (varOrient, varFwd, varLat, covar))

    moveResults = MoveCalibrationResults.fromVals(
        avgDeltaOrient,
        avgDeltaFwd,
        avgDeltaLat,
        varOrient,
        varFwd,
        varLat,
        covar)


    moveResults.errorVarOrient = errorVarOrient
    moveResults.errorVarFwd = errorVarFwd
    moveResults.errorVarLat = errorVarLat
    moveResults.errorCovar = errorCovar

    if verbose == loud:
        print('Results:')
        pprint.pprint(moveResults)

    return moveResults


def calcMoveErrorPerUs(errorVals0, errorValsDict, verbose):

    errorValAvgsList = []
    for i in range(7):
        errorValAvgsList.append([])

    for dist, errorVals in errorValsDict.items():
        for i in range(7):
            # N.B. The distances may be presented ito. ms, but should be output ito. us
            valTuple = (dist * 1000, errorVals[i])
            errorValAvgsList[i].append (valTuple)

    for i in range(7):
        errorValAvgsList[i].sort(key=lambda x: x[0])

    if verbose == loud:
        print('MoveErrorPerUsLists...')
        for i in range(7):
            print(repr(i))
            for index, errorVals in errorValAvgsList[i]:
                print('%d: %s' % (index, repr(errorVals)))

    if verbose == loud:
        print('')
        print('MoveErrorPerUsLists...')
    errorVals = []
    for i in range(7):
        print(repr(i))
        avgIncVal = 0
        for us, errorVal in errorValAvgsList[i]:
            distInc = errorVal - errorVals0[i]
            avgIncVal += distInc/us
            if verbose == loud:
                print('%.15f, %.15f (/%d)' % (distInc, distInc/us, us))
        errorVals.append(avgIncVal)
    if verbose == loud:
        print('')

    print('MovePerUs')
    for i in range(7):
        print('%d:%.15f' % (i, errorVals[i]))
    return errorVals


def calcMoveErrorPerUsNEW(errorVals0, errorValsDict, verbose):
    nVals = 7
    errorValAvgsList = []
    for i in range(nVals):
        errorValAvgsList.append([])

    for dist, errorVals in errorValsDict.items():
        # N.B. The distances may be presented ito. ms, but should be output ito. us
        errorValAvgsList[0].append((dist * 1000, errorVals.avgDeltaOrient))
        errorValAvgsList[1].append((dist * 1000, errorVals.avgDeltaFwd))
        errorValAvgsList[2].append((dist * 1000, errorVals.avgDeltaLat))
        errorValAvgsList[3].append((dist * 1000, errorVals.varOrient))
        errorValAvgsList[4].append((dist * 1000, errorVals.varFwd))
        errorValAvgsList[5].append((dist * 1000, errorVals.varLat))
        errorValAvgsList[6].append((dist * 1000, errorVals.covar))

    for i in range(nVals):
        errorValAvgsList[i].sort(key=lambda x: x[0])

    if verbose == loud:
        print('MoveErrorPerUsLists...')
        for i in range(nVals):
            print(repr(i))
            for index, errorVals in errorValAvgsList[i]:
                print('%d: %s' % (index, repr(errorVals)))

    if verbose == loud:
        print('')
        print('MoveErrorPerUsLists...')

    def __calcErrorPerUs(avgsList, baseVal, valueName):
        print(valueName)
        avgIncs = []
        for us, errorVal in avgsList:
            distInc = errorVal - baseVal
            avgIncs.append(distInc / us)
            if verbose == loud:
                print('%.15f, %.15f (/%d)' % (distInc, distInc/us, us))
        avgPerUs = sum(avgIncs) / len(avgIncs)
        return avgPerUs

    moveResultsPerUs = MoveCalibrationResults()
    moveResultsPerUs.avgDeltaOrient = __calcErrorPerUs(errorValAvgsList[0], errorVals0.avgDeltaOrient, 'avgDeltaOrient')
    moveResultsPerUs.avgDeltaFwd = __calcErrorPerUs(errorValAvgsList[1], errorVals0.avgDeltaFwd, 'avgDeltaFwd')
    moveResultsPerUs.avgDeltaLat = __calcErrorPerUs(errorValAvgsList[2], errorVals0.avgDeltaLat, 'avgDeltaLat')
    moveResultsPerUs.varOrient = __calcErrorPerUs(errorValAvgsList[3], errorVals0.varOrient, 'varOrient')
    moveResultsPerUs.varFwd = __calcErrorPerUs(errorValAvgsList[4], errorVals0.varFwd, 'varFwd')
    moveResultsPerUs.varLat = __calcErrorPerUs(errorValAvgsList[5], errorVals0.varLat, 'varLat')
    moveResultsPerUs.covar = __calcErrorPerUs(errorValAvgsList[6], errorVals0.covar, 'covar')

    if verbose == loud:
        print('')

    print('MovePerUs')
    pprint.pprint(moveResultsPerUs)

    return moveResultsPerUs


moveParamsTemplate = '''
#define ROT_LEFT_DELTA_ORIENT                               %(rotLeftDeltaOrient).15ff
#define ROT_LEFT_DELTA_FWD_WORLD                            %(rotLeftDeltaFwd).15ff
#define ROT_LEFT_DELTA_LAT_WORLD                            %(rotLeftDeltaLat).15ff
#define ROT_LEFT_VAR_ORIENT                                 %(rotLeftVarOrient).15ff
#define ROT_LEFT_VAR_FWD_WORLD                              %(rotLeftVarFwd).15ff
#define ROT_LEFT_VAR_LAT_WORLD                              %(rotLeftVarLat).15ff
#define ROT_LEFT_COVAR_WORLD                                %(rotLeftCovar).15ff

#define ROT_RIGHT_DELTA_ORIENT                              %(rotRightDeltaOrient).15ff
#define ROT_RIGHT_DELTA_FWD_WORLD                           %(rotRightDeltaFwd).15ff
#define ROT_RIGHT_DELTA_LAT_WORLD                           %(rotRightDeltaLat).15ff
#define ROT_RIGHT_VAR_ORIENT                                %(rotRightVarOrient).15ff
#define ROT_RIGHT_VAR_FWD_WORLD                             %(rotRightVarFwd).15ff
#define ROT_RIGHT_VAR_LAT_WORLD                             %(rotRightVarLat).15ff
#define ROT_RIGHT_COVAR_WORLD                               %(rotRightCovar).15ff

#define ROT_AVG_DELTA_ORIENT                                %(rotAvgDeltaOrient).15ff

#define BASE_MOVE_DELTA_ORIENT                              %(baseMoveDeltaOrient).15ff //!< Base orient change in robot for 0ms move.
#define BASE_MOVE_DELTA_FWD_WORLD                           %(baseMoveDeltaFwd).15ff //!< Base move (made with 0ms burst from motors).
#define BASE_MOVE_DELTA_LAT_WORLD                           %(baseMoveDeltaLat).15ff //!< Base offset (perpindicular to direction) for 0ms move.
#define BASE_MOVE_VAR_ORIENT                                %(baseMoveVarOrient).15ff //!< Variance in base orient change measurement.
#define BASE_MOVE_VAR_FWD_WORLD                             %(baseMoveVarFwd).15ff //!< Variance in base fwd move measurement.
#define BASE_MOVE_VAR_LAT_WORLD                             %(baseMoveVarLat).15ff //!< Variance in base lateral move measurement.
#define BASE_MOVE_COVAR_WORLD                               %(baseMoveCovar).15ff //!< Covariance between fwd and lateral move for 0ms burst.

#define MOVE_US_DELTA_ORIENT                                %(moveUsDeltaOrient).15ff //!< Increase in orient change per extra us moved.
#define MOVE_US_DELTA_FWD_WORLD                             %(moveUsDeltaFwd).15ff //!< Increase in dist moved per each extra us motors are run.
#define MOVE_US_DELTA_LAT_WORLD                             %(moveUsDeltaLat).15ff //!< Increase in offset (perp to direction) per each extra us.
#define MOVE_US_VAR_ORIENT                                  %(moveUsVarOrient).15ff //!< Increase in orient change variance per us.
#define MOVE_US_VAR_FWD_WORLD                               %(moveUsVarFwd).15ff //!< Increase in fwd move variance per us.
#define MOVE_US_VAR_LAT_WORLD                               %(moveUsVarLat).15ff //!< Increase in lateral move variance per us.
#define MOVE_US_COVAR_WORLD                                 %(moveUsCovar).15ff //!< Increase in move covariance between us.
'''


def calcMoveParams(measurements):
    settingsDict = {}

    def __calcRotMatForWheelPts(leftPt, rightPt):
        '''
        Calculate matrix to convert from orientation of vector between measured points
        to robot's forward vector.
        '''
        vecBetweenStartPts = Vec3.sub(rightPt, leftPt)
        vecAng = vecBetweenStartPts.getOrient()
        fwdMat = Mat3.setupRotMat(vecAng)
        mat = fwdMat.getInv()
        return mat

    def __calcCentreDistRatioFromLeftPt(leftPt, rightPt):
        '''
        Calculate COG offset from left point as fraction of total dist between measured points.
        '''
        leftLen = leftPt.getLen()
        rightLen = rightPt.getLen()
        centreDistRatioFromLeftPt = leftLen / (leftLen + rightLen)
        return centreDistRatioFromLeftPt

    centreDistRatioFromLeftPt = __calcCentreDistRatioFromLeftPt(
        Vec3(*measurements.rotLeft[0]['locs'][0][0]),
        Vec3(*measurements.rotLeft[0]['locs'][0][1]))

    robotRotMat = __calcRotMatForWheelPts(
        Vec3(*measurements.rotLeft[0]['locs'][0][0]),
        Vec3(*measurements.rotLeft[0]['locs'][0][1]))
    
    # New dict introduced to calculate variance in error, i.e. second order variance
    errorVariance = {'left':{}, 'right':{}, 'fwd':[]}

    # Left turn
    print('Left rotations...')
    deltas = []
    for locDict in measurements.rotLeft:
        deltas.extend(calcMoveDeltasNEW(locDict['locs'], locDict['accumd'], robotRotMat, centreDistRatioFromLeftPt, loud))
    errorVals = calcMoveErrorNEW(deltas, loud)
    settingsDict['rotLeftDeltaOrient'] = errorVals.avgDeltaOrient
    settingsDict['rotLeftDeltaFwd'] = errorVals.avgDeltaFwd
    settingsDict['rotLeftDeltaLat'] = errorVals.avgDeltaLat
    settingsDict['rotLeftVarOrient'] = errorVals.varOrient
    settingsDict['rotLeftVarFwd'] = errorVals.varFwd
    settingsDict['rotLeftVarLat'] = errorVals.varLat
    settingsDict['rotLeftCovar'] = errorVals.covar
    
    errorVariance['left']['errorVarOrient'] = errorVals.errorVarOrient
    errorVariance['left']['errorVarFwd'] = errorVals.errorVarFwd
    errorVariance['left']['errorVarLat'] = errorVals.errorVarLat
    errorVariance['left']['errorCovar'] = errorVals.errorCovar
    print('')

    # Right turn
    print('Right rotations...')
    deltas = []
    for locDict in measurements.rotRight:
        deltas.extend(calcMoveDeltasNEW(locDict['locs'], locDict['accumd'], robotRotMat, centreDistRatioFromLeftPt, loud))
    errorVals = calcMoveErrorNEW(deltas, loud)
    settingsDict['rotRightDeltaOrient'] = errorVals.avgDeltaOrient
    settingsDict['rotRightDeltaFwd'] = errorVals.avgDeltaFwd
    settingsDict['rotRightDeltaLat'] = errorVals.avgDeltaLat
    settingsDict['rotRightVarOrient'] = errorVals.varOrient
    settingsDict['rotRightVarFwd'] = errorVals.varFwd
    settingsDict['rotRightVarLat'] = errorVals.varLat
    settingsDict['rotRightCovar'] = errorVals.covar
    
    errorVariance['right']['errorVarOrient'] = errorVals.errorVarOrient
    errorVariance['right']['errorVarFwd'] = errorVals.errorVarFwd
    errorVariance['right']['errorVarLat'] = errorVals.errorVarLat
    errorVariance['right']['errorCovar'] = errorVals.errorCovar
    print('')

    settingsDict['rotAvgDeltaOrient'] = (math.fabs(settingsDict['rotLeftDeltaOrient']) + math.fabs(settingsDict['rotRightDeltaOrient'])) / 2

    # Fwd
    print('Fwd moves...')
    errorValsDict = {}
    baseErrorVals = None
    for dist, dictList in measurements.fwdMoves.items():
        print('dist %d' % dist)
        deltas = []
        for locDict in dictList:
            deltas.extend(calcMoveDeltasNEW(locDict['locs'], locDict['accumd'], robotRotMat, centreDistRatioFromLeftPt, loud))
        if 0 == dist:
            print('Fwd moves, 0 us...')
            baseErrorVals = calcMoveErrorNEW(deltas, loud)
            settingsDict['baseMoveDeltaOrient'] = baseErrorVals.avgDeltaOrient
            settingsDict['baseMoveDeltaFwd'] = baseErrorVals.avgDeltaFwd
            settingsDict['baseMoveDeltaLat'] = baseErrorVals.avgDeltaLat
            settingsDict['baseMoveVarOrient'] = baseErrorVals.varOrient
            settingsDict['baseMoveVarFwd'] = baseErrorVals.varFwd
            settingsDict['baseMoveVarLat'] = baseErrorVals.varLat
            settingsDict['baseMoveCovar'] = baseErrorVals.covar
            
            tempDict = {}
            tempDict['errorVarOrient'] = baseErrorVals.errorVarOrient
            tempDict['errorVarFwd'] = baseErrorVals.errorVarFwd
            tempDict['errorVarLat'] = baseErrorVals.errorVarLat
            tempDict['errorCovar'] = baseErrorVals.errorCovar
            print('')
        else:
            errorValsDict[dist] = calcMoveErrorNEW(deltas, loud)
            
            tempDict = {}
            tempDict['errorVarOrient'] = baseErrorVals.errorVarOrient
            tempDict['errorVarFwd'] = baseErrorVals.errorVarFwd
            tempDict['errorVarLat'] = baseErrorVals.errorVarLat
            tempDict['errorCovar'] = baseErrorVals.errorCovar
            errorVariance['fwd'].append(tempDict)

    errorValsPerUs = calcMoveErrorPerUsNEW(baseErrorVals, errorValsDict, loud)
    print('Fwd moves, per us...')
    settingsDict['moveUsDeltaOrient'] = errorValsPerUs.avgDeltaOrient
    settingsDict['moveUsDeltaFwd'] = errorValsPerUs.avgDeltaFwd
    settingsDict['moveUsDeltaLat'] = errorValsPerUs.avgDeltaLat
    settingsDict['moveUsVarOrient'] = errorValsPerUs.varOrient
    settingsDict['moveUsVarFwd'] = errorValsPerUs.varFwd
    settingsDict['moveUsVarLat'] = errorValsPerUs.varLat
    settingsDict['moveUsCovar'] = errorValsPerUs.covar
    print('')

    moveParams = moveParamsTemplate % settingsDict
    print(moveParams)

    outfile = '__moveParamsToCopyToCode__.txt'
    open(outfile, 'w').write(moveParams)
    
    
    errorVariance['fwdAvgs'] = {'errorVarOrient': 0, 'errorVarFwd': 0, 'errorVarLat': 0, 'errorCovar': 0}
    errorVariance['fwdAvgs']['errorVarOrient'] = sum(x['errorVarOrient'] for x in errorVariance['fwd'])/len(errorVariance['fwd'])
    errorVariance['fwdAvgs']['errorVarFwd'] = sum(x['errorVarFwd'] for x in errorVariance['fwd'])/len(errorVariance['fwd'])
    errorVariance['fwdAvgs']['errorVarLat'] = sum(x['errorVarLat'] for x in errorVariance['fwd'])/len(errorVariance['fwd'])
    errorVariance['fwdAvgs']['errorCovar'] = sum(x['errorCovar'] for x in errorVariance['fwd'])/len(errorVariance['fwd'])

    errorVariance['final'] = {'errorVarOrient': 0, 'errorVarFwd': 0, 'errorVarLat': 0, 'errorCovar': 0}
    errorVariance['final']['errorVarOrient'] = (errorVariance['left']['errorVarOrient'] + errorVariance['right']['errorVarOrient'] + errorVariance['fwdAvgs']['errorVarOrient'])/3
    errorVariance['final']['errorVarFwd'] = (errorVariance['left']['errorVarFwd'] + errorVariance['right']['errorVarFwd'] + errorVariance['fwdAvgs']['errorVarFwd'])/3
    errorVariance['final']['errorVarLat'] = (errorVariance['left']['errorVarLat'] + errorVariance['right']['errorVarLat'] + errorVariance['fwdAvgs']['errorVarLat'])/3
    errorVariance['final']['errorCovar'] = (errorVariance['left']['errorCovar'] + errorVariance['right']['errorCovar'] + errorVariance['fwdAvgs']['errorCovar'])/3
    
    def __printErrorVar(msg, errorVarDict):
        print(msg)
        d0 = errorVarDict
        print('errorVarOrient:%.3f errorVarFwd:%.3f errorVarLat:%.3f errorCovar:%.3f' % (d0['errorVarOrient'], d0['errorVarFwd'], d0['errorVarLat'], d0['errorCovar']))
    
    printErrorVar = 0
    if printErrorVar:
        __printErrorVar('Error variance for left moves', errorVariance['left'])
        __printErrorVar('Error variance for right moves', errorVariance['right'])
        __printErrorVar('Error variance for fwd moves', errorVariance['fwdAvgs'])
        __printErrorVar('Error variance for all moves', errorVariance['final'])

    print('Move params written to %s' % outfile)


################################################################################################
#
# Main
#
################################################################################################

def calibCompassParams(measurements):

    def __radsToDegs(rads):
        return ((rads * 180.0) / math.pi)

    def __degsToRads(degs):
        return ((degs * math.pi) / 180.0)

    compassValues = measurements.compassValues

    compassTuples = []
    for orientDegs, vals in compassValues.items():
        valsDegs = [__radsToDegs(x) for x in vals]
        mean = sum(valsDegs) / len(valsDegs)
        stdDev = sum(math.fabs(x - mean) for x in valsDegs) / len(valsDegs)
        compassTuple = (orientDegs, mean, stdDev)
        compassTuples.append(compassTuple)

    # Sort by measured orient. Actual orient is arbitrary, so by using measured orient, we can
    # come up with a polynomial to account for error in measured values.
    compassTuples.sort(key=lambda x: x[1])

    if 0:
        for compassTuple in compassTuples:
            print('actual %12f measured %12f, stdDevMeasured %12f' % compassTuple)
        print('')

    # We want to line up actual orient with measured, so use orientDiff which
    # gives orient2 - orient1.
    alignVal = geometryUtils.orientDiff_degs(compassTuples[0][0], compassTuples[0][1])
    print('Align val %12f' % alignVal)

    compassTuplesOld = compassTuples
    compassTuples = []
    for compassTupleOld in compassTuplesOld:
        compassTuple = (geometryUtils.sum_degs(compassTupleOld[0], alignVal), compassTupleOld[1], compassTupleOld[2])
        compassTuples.append(compassTuple)

    if 0:
        for compassTuple in compassTuples:
            print('actual %12f measured %12f, stdDevMeasured %12f' % compassTuple)

    # Format tuples to calculate function to correct error.
    # When running experiments, we will have measured orient and want to determine actual orient from this.
    # So here we are sure that actualOrient is right (that is, after we align orient=0 with our measuredOrient vals)
    # so then we apply a function to the measured orient to try and get it to fit the actual orient better
    print('Measured orient Actual orient')
    diffs = []
    for compassTuple in compassTuples:
        print('%12f %12f' % (compassTuple[1], compassTuple[0]))
        diff = geometryUtils.absOrientDiff_degs(compassTuple[1], compassTuple[0])
        diffs.append(diff)

    avgDiff = sum(diffs) / len(diffs)
    print('Avg diff %12f max diff %12f min diff %12f' % (avgDiff, max(diffs), min(diffs)))

    applyAnyAdjustmentFunc = 0
    if applyAnyAdjustmentFunc:
        def __applyCompassFunction_robot2(inputOrient):
            '''Lorentzian Peak E plus offset. From zunzun.com'''
            a = 0.00135605
            b = 515.31996944
            c = -10653.60550022
            offset = -268.55039660
            outputOrient = (1.0 / (a + math.pow((inputOrient - b) / c, 2.0))) + offset
            return outputOrient

        adjustmentFunc = __applyCompassFunction_robot2
        #adjustmentFunc = asdf
        #adjustmentFunc = qwer

        estimatedTuples = []
        for compassTuple in compassTuples:
            #estimatedOrient = __applyCompassFunction_robot2(compassTuple[1])
            estimatedOrient = adjustmentFunc(compassTuple[1])
            estimatedTuple = (compassTuple[0], estimatedOrient)
            estimatedTuples.append(estimatedTuple)

        print('Estimated orient Actual orient')
        diffs = []
        for estimatedTuple in estimatedTuples:
            print('%12f %12f' % (estimatedTuple[1], estimatedTuple[0]))
            diff = geometryUtils.absOrientDiff_degs(estimatedTuple[1], estimatedTuple[0])
            diffs.append(diff)

        avgDiff = sum(diffs) / len(diffs)
        print('Avg diff %12f max diff %12f min diff %12f' % (avgDiff, max(diffs), min(diffs)))


################################################################################################
#
# Main
#
################################################################################################
import calibrationData

if __name__ == '__main__':
    calibCamera = 0
    calibMoves = 1
    calibCompass = 0

    measurements = calibrationData.Robot2measurements()
    #measurements = calibrationData.Robot3measurements()
    #measurements = calibrationData.Robot4measurements()
 
    if calibCamera:
        #measurements.camTheta = calcCamTheta(measurements)
        #refineCamParams(measurements, quiet)
        #measurements = calcCamThetaNEW(measurements)
        measurements = calcCamThetaNEW2(measurements)
        printCamParams(measurements)

    if calibMoves:
        calcMoveParams(measurements)

    if calibCompass:
        calibCompassParams(measurements)

    print('Done')
    