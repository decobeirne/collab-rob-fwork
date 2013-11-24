import math
from pdb import set_trace



################################################################################################
#
# Geometry
#
################################################################################################

class Vec3(object):
    def __init__(self, _x=0, _y=0, _z=0):
        self.x = _x
        self.y = _y
        self.z = _z
    
    def __repr__(self):
        return '%f,%f,%f' % (self.x, self.y, self.z)
    
    def getNorm(self):
        len = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        return Vec3(self.x / len, self.y / len, self.z / len)
    
    def getLen(self):
        len = self.x * self.x + self.y * self.y + self.z * self.z
        len = math.sqrt(len)
        return len

    @staticmethod
    def dot(vec1, vec2):
        dp = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
        return dp

    @staticmethod
    def cross(vec1, vec2):
        cp = (vec1.y * vec2.z - vec1.z * vec2.y, vec1.z * vec2.x - vec1.x * vec2.z, vec1.x * vec2.y - vec1.y * vec2.x)
        return Vec3(*cp)

    @staticmethod
    def sub(vec1, vec2):
        vec3 = (vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z)
        return Vec3(*vec3)

    def getOrient(self):
        if 0 == self.x and 0 == self.y:
            return 0

        if 0 == self.x:
            if self.y > 0:
                return math.pi * 0.5
            else:
                return math.pi * 1.5

        orient = math.atan(self.y / self.x)
        if orient < 0:
            orient = -orient

        if self.x < 0:
            if self.y > 0:
                orient = math.pi - orient
            else:
                orient = math.pi + orient
        elif self.y < 0:
            orient = (math.pi * 2) - orient
        return orient


class Mat3(object):
    def __init__(self):
        self.vals = [0] * 9
    
    @classmethod
    def fromVals(cls, val0, val1, val2, val3, val4, val5, val6, val7, val8):
        mat = cls()
        mat.vals[0] = val0; mat.vals[1] = val1; mat.vals[2] = val2
        mat.vals[3] = val3; mat.vals[4] = val4; mat.vals[5] = val5
        mat.vals[6] = val6; mat.vals[7] = val7; mat.vals[8] = val8
        return mat
        
    def __repr__(self):
        return '%f,%f,%f\n%f,%f,%f\n%f,%f,%f' % (self.vals[0], self.vals[1], self.vals[2], self.vals[3], self.vals[4], self.vals[5], self.vals[6], self.vals[7], self.vals[8])
    
    def setItem(self, x, y, val):
        index = x + y * 3
        self.vals[index] = val
    
    def getItem(self, x, y):
        index = x + y * 3
        return self.vals[index]

    def getInv(self):
        '''
        http://en.wikipedia.org/wiki/Invertible_matrix
        A^-1 = det(A)|x1 x x2|
                     |x2 x x0|
                     |x0 x x1|
        where x0 is COLUMN 0
        det(A) = triple product = x0 . (x1 x x2)
        '''
        asdf = self.getItem(2, 2)
        vec0 = Vec3(self.getItem(0, 0), self.getItem(1, 0), self.getItem(2, 0))
        vec1 = Vec3(self.getItem(0, 1), self.getItem(1, 1), self.getItem(2, 1))
        vec2 = Vec3(self.getItem(0, 2), self.getItem(1, 2), self.getItem(2, 2))
        
        cross01 = Vec3.cross(vec0, vec1)
        cross12 = Vec3.cross(vec1, vec2)
        cross20 = Vec3.cross(vec2, vec0)
        
        det = Vec3.dot(vec0, cross12)
        det = 1 / det
        
        invMat = Mat3.fromVals(
            det * cross12.x, det * cross12.y, det * cross12.z,
            det * cross20.x, det * cross20.y, det * cross20.z,
            det * cross01.x, det * cross01.y, det * cross01.z)
        return invMat
    
    @staticmethod
    def setupRotMat(ang):
        '''
        rotation mat = cos -sin    0
                       sin  cos    0
                         0    0    1
        '''
        c0 = math.cos(ang)
        s0 = math.sin(ang)
        mat = Mat3.fromVals(c0, -s0, 0, s0, c0, 0, 0, 0, 1)
        return mat
    
    def rotVec(self, vec):
        out0 = self.getItem(0, 0) * vec.x + self.getItem(0, 1) * vec.y + self.getItem(0, 2) * vec.z
        out1 = self.getItem(1, 0) * vec.x + self.getItem(1, 1) * vec.y + self.getItem(1, 2) * vec.z
        out2 = self.getItem(2, 0) * vec.x + self.getItem(2, 1) * vec.y + self.getItem(2, 2) * vec.z
        rotatedVec = Vec3(out0, out1, out2)
        return rotatedVec

# def norm(vec):
    # len = math.sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])
    # normvec = (vec[0]/len, vec[1]/len, vec[2]/len)
    # return normvec


# def dotProd(vec1, vec2):
    # dp = vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
    # return dp


# def crossProd(vec1, vec2):
    # cp = (vec1[1] * vec2[2] - vec1[2] * vec2[1], vec1[2] * vec2[0] - vec1[0] * vec2[2], vec1[0] * vec2[1] - vec1[1] * vec2[0])
    # return cp


# def subVectors(vec1, vec2):
    # vec3 = (vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2])
    # return vec3


# def vectorLen(vec):
    # return math.sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])


def absOrientDiff_rads(orient1, orient2):
    '''
    Absolute orient diff
    '''
    d0 = math.fabs(orient1 - orient2)
    d1 = math.fabs(orient1 - (orient2 + math.pi*2.0))
    d2 = math.fabs((orient1 + math.pi*2.0) - orient2)
    return min(d0, min(d1, d2))


def absOrientDiff_degs(orient1, orient2):
    '''
    Absolute orient diff
    '''
    d0 = math.fabs(orient1 - orient2)
    d1 = math.fabs(orient1 - (orient2 + 360.0))
    d2 = math.fabs((orient1 + 360.0) - orient2)
    return min(d0, min(d1, d2))


def orientDiff_rads(orient1, orient2):
    d0 = orient2 - orient1;

    # if the difference is greater than 180, then the angles are either
    # side of the origin
    if math.fabs(d0) > math.pi:
        temp1 = orient1
        temp2 = orient2

        # if orient1 is to the right of the origin, then add 359 to this such
        # that the intest difference between the orientations can be calculated
        if temp1 < math.pi:
            temp1 += math.pi * 2.0
        else:
            temp2 += math.pi * 2.0

        d0 = temp2 - temp1;
    return d0


def orientDiff_degs(orient1, orient2):
    d0 = orient2 - orient1;

    # if the difference is greater than 180, then the angles are either
    # side of the origin
    if math.fabs(d0) > 180.0:
        temp1 = orient1
        temp2 = orient2

        # if orient1 is to the right of the origin, then add 359 to this such
        # that the intest difference between the orientations can be calculated
        if temp1 < 180.0:
            temp1 += 360.0
        else:
            temp2 += 360.0

        d0 = temp2 - temp1;
    return d0    


def sum_rads(orient1, orient2):
    orientSum = orient1 + orient2
    if orientSum < 0.0:
        orientSum = (math.pi * 2.0) + orientSum
    elif orientSum >= (math.pi * 2.0):
        orientSum = orientSum - (math.pi * 2.0)
    return orientSum


def sum_degs(orient1, orient2):
    orientSum = orient1 + orient2
    if orientSum < 0.0:
        orientSum = 360.0 + orientSum
    elif orientSum >= 360.0:
        orientSum = orientSum - 360.0
    return orientSum


# def setupRotMat3D(ang):
    # '''
    # rotation mat = cos -sin    0
                   # sin  cos    0
                     # 0    0    1
    # '''
    # c0 = math.cos(ang)
    # s0 = math.sin(ang)
    # mat = (c0, -s0, 0, s0, c0, 0, 0, 0, 1)
    # return mat


# def rotVec3D(vec, mat):
    # out0 = mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2]
    # out1 = mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2]
    # out2 = mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2]
    # return (out0, out1, out2)


# def matInv3D(mat):
    # '''
    # http://en.wikipedia.org/wiki/Invertible_matrix
    # A-1 = det(A)|x1 x x2|
                # |x2 x x0|
                # |x0 x x1|
    # where x0 is COLUMN 0
    # det(A) = triple product = x0 . (x1 x x2)
    # '''
    # x0 = (mat[0], mat[3], mat[6])
    # x1 = (mat[1], mat[4], mat[7])
    # x2 = (mat[2], mat[5], mat[8])
    # v0 = crossProd(x1, x2)
    # det = dotProd(x0, v0)
    # det = 1 / det

    # v1 = crossProd(x2, x0)
    # v2 = crossProd(x0, x1)

    # mat = (det*v0[0], det*v0[1], det*v0[2], det*v1[0], det*v1[1], det*v1[2], det*v2[0], det*v2[1], det*v2[2])
    # return mat


# def calcAxisRotationMat(loc):
    # '''
    # Location should be given as (left,right) or (frontLeft,backRight)
    # '''
    # x = loc[0][0] - loc[1][0]
    # y = loc[0][1] - loc[1][1]
    # ang = math.atan2(y, x)
    # mat = setupRotMat3D(ang)
    # matInv = matInv3D(mat)
    # return matInv


################################################################################################
#
# Testing
#
################################################################################################


def testGeom1():
    vec1 = (4.0,2.0,1.0)
    vec2 = (2.5,2.0,3.0)

    normVec = norm(vec1)
    print('norm ' + str(normVec))

    dp = dotProd(normVec, vec2)
    print('dot ' + str(dp))

    cp = crossProd(vec1, vec2)
    print('cross ' + str(cp))

def testGeom2():
    vec = (8, 5, 0)
    ang = math.pi/6.0
    mat = setupRotMat3D(ang)

    for i in range(13):
        vec2 = rotVec3D(vec, mat)
        vec = vec2
        print(vec)

    print('inv')
    mat2 = matInv3D(mat)

    for i in range(13):
        vec2 = rotVec3D(vec, mat2)
        vec = vec2
        print(vec)


if __name__ == '__main__':
    testGeom1()
    testGeom2()
