import numpy as np
import cv2 as cv

# ARUCO_DICT = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)
# ARUCO_PARAMS = cv.aruco.DetectorParameters_create()


# def generate_charuco(board_width, board_height):
#     return cv.aruco.CharucoBoard_create(board_width, board_height, 0.04, 0.02, ARUCO_DICT)


# def detect_markers(img):
#     return cv.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMS)


# def draw_markers(img):
#     imgDetected = img.copy()
#     imgRejected = img.copy()
#     (corners, ids, rejected) = detect_markers(img)
#     cv.aruco.drawDetectedMarkers(imgDetected, corners, ids)
#     cv.aruco.drawDetectedMarkers(
#         imgRejected, rejected, borderColor=(100, 0, 240))
#     return (imgDetected, imgRejected)


# def set_params(params):
#     if params.getNode('adaptiveThreshConstant') != None:
#         ARUCO_PARAMS.adaptiveThreshConstant = float(
#             params.getNode('adaptiveThreshConstant'))


ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)


def detect_aruco(img: np.ndarray):
    return cv.aruco.ArucoDetector(ARUCO_DICT).detectMarkers(img)


def detect_contours(img: np.ndarray):
    pass
