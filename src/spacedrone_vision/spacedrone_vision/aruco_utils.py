'''
ArUco helper functions, pulled out of the old aruco.py / aruco_publisher.py so they can
be used by the ROS node AND tested without ROS running.

Original detection + pose code was based on
https://github.com/niconielsen32/ComputerVision/tree/master/ArUco
'''

import math
import cv2
import numpy as np


# dictionary for tags (same as before, just trimmed down to the ones we'd actually use)
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class TagDetector:
    '''
    Wrapper because opencv changed the aruco API in 4.7 (ArucoDetector class) and
    the apt version on ubuntu 24.04 is 4.6 which only has the old free functions.
    Laptops on the team had both so this handles either.
    '''

    def __init__(self, dict_name="DICT_4X4_50"):
        if dict_name not in ARUCO_DICT:
            raise ValueError(f"unknown aruco dict {dict_name}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])

        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self._new_api = True
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._new_api = False

        # subpixel refinement makes the pose a LOT less jittery from altitude
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # params.adaptiveThreshWinSizeMax = 53   # tried this for shadows, didnt really help

        # if the white border around the tag is thin, aruco finds the outer white edge AND the
        # inner black edge as 2 candidates, decides they're "too close" and keeps the wrong one.
        # so tags on a grey floor were just never detected.
        # params.minMarkerDistanceRate = 0.02
        # ^ "fixes" it but then the outer edge sometimes decodes as the same id and you get the
        # tag twice with a pose ~5% off. real fix = print the tags with a fat white border
        # (at least ~20% of the marker width). generate_tags.py does this now
        self.params = params

        if self._new_api:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)

    def detect(self, gray):
        if self._new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        return corners, ids, rejected


# calculate pose of the detected tags (same idea as the old pose_estimation function)
def pose_estimation(corners, marker_size, matrix_coefficients, distortion_coefficients):

    # define 3d object points, order has to match the corner order aruco gives:
    # top left, top right, bottom right, bottom left
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    ok_list = []

    # IPPE_SQUARE is made specifically for square markers
    for c in corners:
        ok, R, t = cv2.solvePnP(marker_points, c, matrix_coefficients, distortion_coefficients,
                                False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        ok_list.append(ok)

    # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, matrix_coefficients, distortion_coefficients)
    # ^ this is deprecated and gave flipped z axis sometimes, solvePnP is better

    return rvecs, tvecs, ok_list


def rvec_to_quaternion(rvec):
    '''rodrigues vector -> (x, y, z, w) quaternion'''
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    # standard rotation matrix -> quaternion
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


# determine the area of the tag in pixels (shoelace formula, the bounding box thing
# from the yolo code doesnt work well when the tag is rotated)
def tag_pixel_area(corner):
    pts = corner.reshape((4, 2))
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def intrinsics_from_fov(width, height, hfov):
    '''
    fallback camera matrix if camera_info never shows up. gazebo cameras are
    perfect pinholes so this is exact for the sim (hfov is in radians)
    '''
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return np.array([[fx, 0, width / 2.0],
                     [0, fx, height / 2.0],
                     [0, 0, 1]], dtype=np.float64)
