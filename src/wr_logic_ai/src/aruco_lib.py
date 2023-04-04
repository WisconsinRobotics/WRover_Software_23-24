import numpy as np
import cv2 as cv

ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
ARUCO_DETECTOR = cv.aruco.ArucoDetector(ARUCO_DICT)
SIDE_LENGTH_1FT = 675
FT_TO_M = 0.3048

def detect_aruco(img: np.ndarray):
    return ARUCO_DETECTOR.detectMarkers(img)


def mask_black(img: np.ndarray):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    black_thresh = cv.inRange(hsv, (40, 0, 0), (180, 255, 120))
    return black_thresh


def mask_white(img: np.ndarray):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    white_thresh = cv.inRange(hsv, (30, 0, 135), (180, 30, 255))
    return white_thresh


def combine_masks(mask_a: np.ndarray, mask_b: np.ndarray, kernel_size: int=5):
    kernel = np.ones((kernel_size, kernel_size), np.int8)
    mask_a_convolution = cv.filter2D(mask_a, -1, kernel)
    mask_b_convolution = cv.filter2D(mask_b, -1, kernel)
    and_convolutions = cv.bitwise_and(mask_a_convolution, mask_b_convolution)
    return and_convolutions


def gaussian_blur_discrete(img: np.ndarray, kernel_size: int=5):
    blur = cv.GaussianBlur(img, (kernel_size, kernel_size), 20)
    discrete_blur = cv.inRange(blur, 1, 255)
    return discrete_blur


def find_contours(img_mask: np.ndarray, epsilon=3, min_area=250):
    contours, hierarchy = cv.findContours(img_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    rectangular_contours = []
    for contour in contours:
        approx = cv.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4:
            contour_area = cv.contourArea(approx)
            if contour_area > min_area:
                rectangular_contours.append(approx)

    return rectangular_contours


def detect_contours(img: np.ndarray):
    img_mask_black = mask_black(img)
    img_mask_white = mask_white(img)
    img_masks_combined = combine_masks(img_mask_black, img_mask_white)

    contours = find_contours(img_masks_combined)
    return contours


def estimate_distance_ft(corners: np.ndarray):
    side_lengths = [ np.linalg.norm(corners[i-1] - corners[i]) for i in range(len(corners))]
    return SIDE_LENGTH_1FT / max(side_lengths)


def estimate_distance_m(corners: np.ndarray):
    return FT_TO_M * estimate_distance_ft(corners)
 