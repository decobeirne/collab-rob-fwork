
# Code to fit graph to a set of points. From zunzun.com
'''
// To the best of my knowledge this code is correct.
// If you find any errors or problems please contact
// me directly using zunzun@zunzun.com.
//
//      James


#include <math.h>

// Fitting target: lowest sum of squared absolute error
// Fitting target value = 124.813331224

double SimpleEquation_41_Offset_model(double x_in)
{
    double temp;
    temp = 0.0;

    // coefficients
    double a = -6.4270043793288107E+02;
    double b = -6.3726269747699746E+01;
    double c = 1.6604916902772022E+00;
    double Offset = -3.4272237720022432E+01;

    temp = a*pow(x_in,b/x_in)+c*x_in;
    temp += Offset;
    return temp;
}
'''

# Template for recording rotation moves
'''
            {
            'accumd': False,
            'locs': [
                ((, ), (, )),
                ]
            },
'''

# Template for recording fwd moves
'''
            0: [
                {
                'accumd': True,
                'locs': [
                    ]
                },
                ],
'''


class Robot2measurements:
    def __init__(self):
        #
        # Misc. Verified on 2011/12/11
        #
        self.flipv = 1
        self.fliph = 0
        self.flipCompass = 1
        self.hBridge = 1
        self.hmc6352 = 1
        self.switchMotors = 1
        self.origx = 2
        self.origy = 4
        self.camOccupGridy = 23

        #
        # Camera
        #

        # Updated 20130427
        temp = 109
        self.camThetaPixelVals = [
            (150- temp, 74, 10),
            (200- temp, 74, 101),
            (250- temp, 74, 134),
            (250- temp, 0, 31),
            (300- temp, 0, 64),
            (350- temp, 0, 89),
            (400- temp, 0, 105),
            (450- temp, 0, 117),
            ]

        self.camMidptPixel = [88, 71]
        self.camHeight = 111
        self.pDist = temp
        self.focalLen = 253.759170
        self.camTheta = 0.499264

        #
        # Move calibration
        #

        # Left move with 0 us burst. Initial orientation is 0 rads.
        self.rotLeft = [
            {
            'accumd': False,
            'locs': [
                ((0, 66), (0, -69)),
                ((-48, 48), (51, -46)),
                ((-47, 47), (49, -49)),
                ((-52, 49), (46, -46)),
                ]
            },
            {
            'accumd': True,
            'locs': [
                ((0, 66), (0, -69)),
                ((-47, 47), (52, -46)),
                ((-64, -10), (69, 9)),
                ((-39, -49), (42, 42)),
                ((10, -57), (-17, 74)),
                ((48, -30), (-66, 44)),
                ((53, 23), (-77, -14)),
                ((17, 66), (-35, -59)),
                ]
            },
            ]

        self.rotRight = [
            {
            'accumd': True,
            'locs': [
                ((0, 66), (0, -69)),
                ((46, 45), (-56, -45)),
                ((59, -2), (-74, 15)),
                ((36, -41), (-45, 65)),
                ((-15, -52), (12, 79)),
                ((-56, -24), (60, 46)),
                ((-60, 32), (65, -18)),
                ((-26, 67), (23, -58)),
                ]
            },
            ]



        self.fwdMoves = {
            0: [  # Engine burst of 0 us
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((51, 64), (51, -71)),
                    ((104.5, 62.5), (105, -72)),
                    ((157.5, 62.5), (158.5, -73)),
                    ((210.5, 61), (213, -74)),
                    ((262, 60.5), (266, -74)),
                    ((319, 60), (321, -74.5)),
                    ((374, 60), (374.5, -75)),
                    ((426, 60), (427, -75.5)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((56.5, 64.5), (57, -70,5)),
                    ((115.5, 64.5), (116, -70.5)),
                    ((172, 64), (174.5, -71)),
                    ((230, 64), (233.5, -71)),
                    ((290, 64), (292, -70.5)),
                    ((348, 65), (352, -70)),
                    ((406, 64), (411, -68.5)),
                    ((464.5, 68), (470, -62)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((59, 64), (58.5, -71)),
                    ((117, 63.5), (116.5, -71)),
                    ((176, 62.5), (177, -73)),
                    ((235, 61), (235, -74)),
                    ((294, 60.5), (295, -74)),
                    ((353, 60.5), (354.5, -74.5)),
                    ((410, 60.5), (413, -74.5)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((59.5, 64.5), (59, -70)),
                    ((117, 64), (117, -71)),
                    ((177, 63), (176.5, -72)),
                    ((234.5, 61.5), (235, -73)),
                    ((294, 61), (295, -73.5)),
                    ((354, 60.5), (355, -74.5)),
                    ((413.5, 61), (415, -76)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((59, 64), (58.5, -71)),
                    ((120, 63.5), (119, -71)),
                    ((180, 62), (180, -73)),
                    ((238.5, 61), (239, -73.5)),
                    ((299, 60), (299, -75)),
                    ((358.5, 59.5), (358.5, -75)),
                    ((417, 59), (417, -76)),
                    ]
                },

                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((57, 64), (59.5, -71)),
                    ((117.5, 64.5), (119, -71)),
                    ((176, 63.5), (177, -71.5)),
                    ((235, 63), (237, -73)),
                    ((294, 63.5), (299, -72)),
                    ((358, 63.5), (358, -71)),
                    ]
                },
                ],
            50: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((75, 64), (70.5, -70.5)),
                    ((151, 63), (152.5, -72)),
                    ((229, 62), (229, -73)),
                    ((306.4, 60), (306, -74.5)),
                    ((381, 58.5), (381, -76)),
                    ((457.5, 58), (457.5, -77)),
                    ((532.5, 57), (534, -79)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((82.5, 63), (79, -72.5)),
                    ((167, 57), (157, -73.5)),
                    ((249, 49), (235, -85)),
                    ((329, 39), (314, -95)),
                    ]
                },
                ],
            100: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((104, 61), (99, -73)),
                    ((202.5, 54), (197, -81)),
                    ((311.5, 42), (295, -92)),
                    ((415, 26), (393, -107)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((103, 62), (99, -73)),
                    ((206, 55), (197, -80)),
                    ((312, 43), (296, -90)),
                    ((415.5, 28), (395, -106)),
                    ]
                },
                ],
            200: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((115, 65), (118.5, -70)),
                    ((232, 67), (238, -68)),
                    ((352, 70.5), (360, -64.5)),
                    ((474, 76.5), (483, -59)),
                    ((594.5, 83), (606, -51)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((139, 59.5), (134, -75.5)),
                    ((278, 48), (270, -83)),
                    ((416, 41), (405, -95)),
                    ((553, 27), (540, -108)),
                    ]
                },
                ],
            400: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((168.5, 62), (173, -73)),
                    ((338, 57), (336.5, -77)),
                    ((508, 54.5), (511, -81.5)),
                    ((682, 53), (684, -82.5)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((186.5, 63), (189, -71)),
                    ((376.5, 63), (380, -72)),
                    ((561, 65.5), (567, -69)),
                    ((749, 71), (757, -64)),
                    ]
                },
                ],
            600: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((216.5, 65), (221, -70)),
                    ((434, 70.5), (442.5, -64)),
                    ((646, 87), (664, -47)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((224, 61), (228.5, -74)),
                    ((446, 64.5), (455, -79)),
                    ((665.5, 80), (682, -54)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((222, 65), (230, -70)),
                    ((445, 68), (459, -43.5)),
                    ((665.5, 103), (688, -30)),
                    ]
                },
                ],
            800: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((267, 77), (275, -68)),
                    ((531, 86), (549, -48)),
                    ((793.5, 123.5), (819, -8)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((271, 69), (275, -72)),
                    ((532.5, 75.5), (551, -69)),
                    ((803, 102), (821.5, -32.5)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -69)),
                    ((264, 64), (271, -71)),
                    ((534, 72), (544, -73)),
                    ((802, 87), (815, -47)),
                    ]
                },
                ],
            }

        # These were points from when the compass calibration process was run, but before a curve was fit to
        # the points. The measured orients are from an arbitrary origin, but processCalibrationData.py calculates
        # the adjustment such that measured and actual orient line up at 0
        #
        # This calibration data applies to the camera module defined with IS_HMC6352 that is on robot2/robot3
        #
        # The compass module should be calibrated before gathering these values.
        # To do this run remoteControl() with CALIB_COMPASS_START and then CALIB_COMPASS_STOP.
        # These call bCMU_CompassCalibStart and bCMU_CompassCalibExit respectively.
        # These send a 'C' value and 'E' value to start and stop compass calibration...
        # bI2C_WriteByteShort(0x21, 67);
        # bI2C_WriteByteShort(0x21, 69);
        #
        # Instructions for calibration are taken from docs/reference/hardware/newCompass__HMC6352/HMC6352.pdf
        # under the section 'User Calibration' on page 7
        '''
        The HMC6352 provides a user calibration routine with the C command permitting entry into the calibration mode and the
        E command to exit the calibration mode. Once in calibration mode, the user is requested to rotate the compass on a flat
        surface at least one full circular rotation while the HMC6352 collects several readings per second at various headings with
        the emphasis on rotation smoothness to gather uniformly spaced readings. Optimally two rotations over 20 seconds
        duration would provide an accurate calibration. The calibration time window is recommended to be from 6 seconds up to
        3 minutes depending on the end users platform.
        The calibration routine collects these readings to correct for hard-iron distortions of the earths magnetic field. These hardiron
        effects are due to magnetized materials nearby the HMC6352 part that in a fixed position with respect to the end user
        platform. An example would be the magnetized chassis or engine block of a vehicle in which the compass is mounted
        onto. Upon exiting the calibration mode, the resulting magnetometer offsets and scaling factors are updated
        '''
        self.compassValues = {
            0: [
                4.434881,
                4.424409,
                4.424409,
                4.433136,
                4.429646,
                4.438372,
                4.448844,
                4.447099,
                ],
            11.25: [
                4.565781,
                4.565781,
                ],
            22.5: [
                4.731587,
                4.717625,
                ],
            33.75: [
                4.862487,
                4.867723,
                ],
            45: [
                5.016076,
                5.019567,
                ],
            56.25: [
                5.169665,
                5.181882,
                ],
            67.5: [
                5.330235,
                5.331981,
                ],
            78.75: [
                5.497787,
                5.506514,
                ],
            90: [
                5.688028,
                5.668829,
                ],
            101.25: [
                5.866051,
                5.874778,
                ],
            112.5: [
                6.091199,
                6.089454,
                ],
            123.75: [
                0.087266,
                0.050614,
                ],
            135: [
                0.303687,
                0.296706,
                ],
            146.25: [
                0.617846,
                0.612610,
                ],
            157.5: [
                0.877900,
                0.863938,
                ],
            168.75: [
                1.144936,
                1.130973,
                ],
            180: [
                1.401499,
                1.404990,
                ],
            191.25: [
                1.617920,
                1.628392,
                ],
            202.5: [
                1.830850,
                1.829105,
                ],
            213.75: [
                2.026327,
                2.042035,
                ],
            225: [
                2.220058,
                2.225295,
                ],
            236.25: [
                2.415535,
                2.408554,
                ],
            247.5: [
                2.586578,
                2.586578,
                ],
            258.75: [
                2.783800,
                2.769837,
                ],
            270: [
                2.961823,
                2.984513,
                ],
            281.25: [
                3.173008,
                3.169518,
                ],
            292.5: [
                3.368485,
                3.364995,
                ],
            303.75: [
                3.569198,
                3.572689,
                ],
            315: [
                3.775147,
                3.764675,
                ],
            326.25: [
                3.977605,
                3.968879,
                ],
            337.5: [
                4.150393,
                4.152138,
                ],
            348.75: [
                4.291764,
                4.302237,
                ],
            }





class Robot3measurements:
    def __init__(self):
        #
        # Misc. Verified on 2011/12/11.
        #
        self.flipv = 1
        self.fliph = 0
        self.flipCompass = 1
        self.hBridge = 1
        self.hmc6352 = 1
        self.switchMotors = 0
        self.origx = 2
        self.origy = 4
        self.camOccupGridy = 23

        #
        # Camera
        #

        # Compass params. Updated on 20130427 as results from cooperative localisation were quite large. These
        # measurements include pts not on the ground plane.
        temp = 134
        self.camThetaPixelVals = [
            ((149-temp)+119, 0, 32),
            ((149-temp)+150, 0, 51),
            ((149-temp)+250, 0, 92),
            ((149-temp)+350, 0, 113),
            ((149-temp)+450, 0, 126),
            ((149-temp)+550, 0, 135),
            ]  # From robot3_calib_20130210
        '''self.camThetaPixelVals = [
            (200-temp, 72, 86),
            (250-temp, 72, 119),
            (300-temp, 72, 135),
            (250-temp, 0, 21),
            (300-temp, 0, 56),
            (350-temp, 0, 78),
            (400-temp, 0, 94),
            (450-temp, 0, 106),
            (000-temp, 0, 000),
            ]'''  # From robot3_calib_20130228 - not sure about dists

        # Camera parameters. Updated (using refine func in script) from initial estimate/mesaured vals on 2013/02/19.
        self.camMidptPixel = [88, 71]
        self.camHeight = 109
        self.focalLen = 207.292641
        self.camTheta = 0.49
        self.pDist = temp

        #
        # Move calibration. From 2013/02/10.
        #

        self.rotLeft = [
            {
            'accumd': True,
            'locs': [
                ((0, 66), (0, -65)),
                ((-32, 52), (53, -50)),
                ((-47, 17), (83, -5)),
                ((-47, -20), (65, 50)),
                ((-8, -46), (43, 77)),
                ((40, -43), (-17, 76)),
                ((69, -5), (-52, 50)),
                ((71, 33), (-56, 1)),
                ]
            },
            ]

        self.rotRight = [
            {
            'accumd': True,
            'locs': [
                ((0, 66), (0, -65)),
                ((43, 53), (-43, -48)),
                ((65, 15), (-64, -9)),
                ((58, -32), (-53, 37)), #((58, -32), (-53, 47)),
                ((18, -63), (-14, 65)),
                ((-23, -60), (31, 60)),
                ((-59, -22), (64, 26)),
                ((-60, 25), (62, -22)),
                ((-34, 61), (31, -54)), #((-44, 61), (31, -54)),
                ((28, 68), (-19, -58)),
                ((55, 44), (-55, -30)),
                ]
            },
            ]

        self.fwdMoves = {
            0: [  # Engine burst of 0 us
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((37, 66), (39, -65)),
                    ((81 ,67), (86, -65)),
                    ((126, 67), (133, -64)),
                    ((177, 70), (183, -61)),
                    ((219, 73), (230, -58)),
                    ((262, 76), (277, -54)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((52, 66), (54, -65)),
                    ((103, 67), (109, -64)),
                    ((151, 68), (165, -63)),
                    ((209, 71), (220, -60)),
                    ((267, 74), (276, -56)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((53, 66), (57, -65)),
                    ((107, 67), (114, -64)),
                    ((158, 69), (168, -62)),
                    ((211, 72), (223, -58)),
                    ((263, 77), (276, -54)),
                    ((316, 82), (331, -50)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((52, 66), (56, -65)),
                    ((106, 67), (116, -64)),
                    ((161, 69), (170, -61)),
                    ((215, 72), (228, -58)),
                    ((269, 77), (285, -53)),
                    ]
                },
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((51, 67), (54, -64)),
                    ]
                },
                ],

            50: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((72, 67), (75, -64)),
                    ((132, 68), (149, -63)),
                    ((214, 70), (223, -60)),
                    ((287, 75), (299, -56)),
                    ((359, 80), (374, -52)),
                    ((433, 88), (450, -43)),
                    ]
                },
                ],

            100: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((88, 67), (91, -64)),
                    ((174, 68), (180, -62)),
                    ((261, 73), (270, -68)),
                    ((350, 77), (359, -54)),
                    ((438, 82), (450, -50)),
                    ((526, 89), (540, -42)),
                    ]
                },
                ],

            200: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((116, 65), (118, -64)),
                    ((230, 66), (235, -65)),
                    ((342, 70), (348, -61)),
                    ((457, 73), (465, -58)),
                    ((572, 80), (583, -50)),
                    ]
                },
                ],

            400: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((165, 65), (160, -67)),
                    ((333, 55), (321, -76)),
                    ((502, 37), (486, -93)),
                    ]
                },
                ],

            600: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((211, 63), (203, -68)),
                    ((428, 42), (413, -88)),
                    ((642, 10), (620, -119)),
                    ]
                },
                ],

            800: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((260, 60), (254, -72)),
                    ((521, 47), (504, -94)),
                    ((782, -6), (756, -136)),
                    ]
                },
                ],

            1000: [
                {
                'accumd': True,
                'locs': [
                    ((0, 66), (0, -65)),
                    ((357, 58), (348, -74)),
                    ((665, 25), (645, -105)),
                    ((971, -35), (941, -161)),
                    ]
                },
                ],
            }

        #
        # Compass
        #


class Robot4measurements:
    def __init__(self):
        #
        # Misc. Updated on 2013/03/10
        #

        self.flipv = 1
        self.fliph = 0
        self.origx = 2
        self.origy = 4
        self.flipCompass = 1
        self.hBridge = 0
        self.hmc6352 = 0
        self.switchMotors = 1 # This robot isn't setup with a h-bridge, so it has a different code path for activating motors.
        self.camOccupGridy = 23

        #
        # Camera
        #

        # Dist/pixel values.
        temp = 116
        self.camThetaPixelVals = [
            (150-temp,71,8),
            (200-temp,71,107),
            (250-temp,71,139),
            (250-temp,0,30),
            (300-temp,0,67),
            (350-temp,0,91),
            (400-temp,0,107),
            (450-temp,0,120),
        ]

        # Camera parameters.
        self.camMidptPixel = [88, 71]
        self.camHeight = 101
        self.pDist = temp
        self.focalLen = 251.211250
        self.camTheta = 0.484952

        #
        # Move calibration. From 2013/03/10.
        #

        self.rotLeft = [
            {
            'accumd': True,
            'locs': [
                ((0,85), (0,-85)),
                ((-22,96), (33,-71)),
                ((-43,95), (60,-45)),
                ((-66,88), (77,-12)),
                ((-85,74), (82,22,)),
                ((-94,54), (80,44)),
                ((-100,37), (70,77)),
                ((-102,14), (52,100)),
                ((-94,-11), (20,121)),
                #((-91,-26), (-3,133)),
                #((-79,-43), (132,-54)),
                #((-58,-52), (129,-33)),
                ]
            },
            ]

        self.rotRight = [
            {
            'accumd': True,
            'locs': [
                ((0,85), (0,-85)),
                ((24,78), (-27,-90)),
                ((47,65), (-47,-86)),
                ((60,40), (-78,-25)),
                ((72,13), (-88,-60)),
                ((75,-16), (-100,-33)),
                ((67,-52), (-101,-4)),
                ((51,-76), (-97,18)),
                ((35,-95), (-85,32)),
                #((-35,-95), (-85,32)),
                ]
            },
            ]


        self.fwdMoves = {
            0: [  # Engine burst of 0 us
                {
                'accumd': True,
                'locs': [
                    ((0,90), (0,-87)), # Cards were changed, and dists are now slightly different
                    ((45, 92), (41, -85)),
                    ((83, 90), (79, -87)),
                    ((123, 89), (119, -87)),
                    ((160, 88), (155, -88)),
                    ((199, 87), (193, -90)),
                    ((245, 86), (233, -91)),
                    ]
                },
                ],
            50: [  # Engine burst of 50 us
                {
                'accumd': True,
                'locs': [
                    ((0,90), (0,-87)),
                    ((48, 90), (48, -86)),
                    ((96, 93), (97, -85)),
                    ((143, 92), (145, -84)),
                    ((190, 93), (192, -84)),
                    ((239, 94), (230, -84)),
                    ]
                },
                ],
            100: [
                {
                'accumd': True,
                'locs': [
                    ((0,90), (0,-87)),
                    ((56, 91), (60, -86)),
                    ((114, 94), (119, -84)),
                    ((169, 93), (174, -83)),
                    ((228, 95), (233, -80)),
                    ((287, 98), (293, -78)),
                    ]
                },
                ],
            400: [
                {
                'accumd': True,
                'locs': [
                    ((0,90), (0,-87)),
                    ((123, 91), (117, -86)),
                    ((234, 91), (235, -86)),
                    ((352, 88), (348, -88)),
                    ((467, 87), (464, -89)),
                    ((585, 85), (580, -91)),
                    ((697, 82), (691, -92)),
                    ]
                },
                ],
            800: [
                {
                'accumd': True,
                'locs': [
                    ((0,85), (0,-85)),
                    ((190, 91), (178, -84)),
                    ((378, 76), (361, -100)),
                    ((567, 53), (532, -120)),
                    ]
                },
                ],
            1000: [
                {
                'accumd': True,
                'locs': [
                    ((0,90), (0,-87)),
                    ((229, 93), (229, -84)),
                    ((457, 103), (462, -73)),
                    ((691, 110), (692, -76)),
                    ]
                },
                ],
            1200: [
                {
                'accumd': True,
                'locs': [
                    ((0,90), (0,-87)),
                    ((266, 98), (273, -78)),
                    ((538, 113), (550, -63)),
                    ((813, 131), (822, -46)),
                    ]
                },
                ],











            }


        #
        # Compass
        #

        # This robot is equipped with a CMPS03.
        # Info on this compass is found in docs/reference/hardware/oldCompass__CMPS03/CMPS03 documentation.htm
        # Calibration is initiated by writing 0xFF to the command register, 15
        #
        # This corresponds to reading the compass in bCMU_ReadCompass, where we call
        # bI2C_ReadWord((BYTE)0x60, (UINT16)0x01, UINT16 *pun16Data); to access address 0x60 on the I2C bus
        # and read register 0x01
        #
        # The factory calibration is for 67 degrees latitude.
        # Calibration instructions are under the section 'I2C Method'
        '''
        Before calibrating the compass, you must know exactly which direction is North, East, South and West.
        Remember these are the magnet poles, not the geographic poles. Don't guess at it.
        Get a magnetic needle compass and check it.
        When calibrating, make sure the compass is horizontal at all times with components upwards, don't tilt it.
        Keep all magnetic and ferrous materials away from the compass during calibration - including your wristwatch.

        I2C Method
        To calibrate using the I2C bus, you only have to write 255 (0xff) to register 15, once for each of the four major compass points North, East, South and West. The 255 is cleared internally automatically after each point is calibrated. The compass points can be set in any order, but all four points must be calibrated. For example
        1. Set the compass module flat, pointing North. Write 255 to register 15, Calibrating pin (pin5) goes low.
        2. Set the compass module flat, pointing East. Write 255 to register 15,
        3. Set the compass module flat, pointing South. Write 255 to register 15,
        4. Set the compass module flat, pointing West. Write 255 to register 15, Calibrating pin (pin5) goes high.
        That's it.
        '''
