##@defgroup wr_shortrange_ai
# @{
# @defgroup wr_shortrange_ai_aruco ArUco helper functions
# @brief Functions for detecting ArUco tags
# @{

import numpy as np
import cv2 as cv

## Initialize ArUco 4x4 marker dictionary
ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
## Initialize ArUco detector
ARUCO_DETECTOR = cv.aruco.ArucoDetector(ARUCO_DICT)
## Constant for converting pixels to feet, may need to be tuned
SIDE_LENGTH_1FT = 675
## Constant for converting feet to meters
FT_TO_M = 0.3048


def detect_aruco(img: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Detects ArUco markers

    @param img (np.ndarray): An OpenCV image
    @return Tuple[np.ndarray, np.ndarray, np.ndarray]: A tuple containing the corners, ids, and rejected points in the image
    """
    return ARUCO_DETECTOR.detectMarkers(img)


def mask_black(img: np.ndarray) -> np.ndarray:
    """
    Create a mask for black pixels

    @param img (np.ndarray): An OpenCV image
    @return np.ndarray: A mask for black pixels in the image
    """
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    black_thresh = cv.inRange(hsv, (40, 0, 0), (180, 255, 120))
    return black_thresh


def mask_white(img: np.ndarray) -> np.ndarray:
    """
    Create a mask for white pixels

    @param img (np.ndarray): An OpenCV image
    @return np.ndarray: A mask for white pixels in the image
    """
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    white_thresh = cv.inRange(hsv, (30, 0, 135), (180, 30, 255))
    return white_thresh


def combine_masks(
    mask_a: np.ndarray, mask_b: np.ndarray, kernel_size: int = 5
) -> np.ndarray:
    """
    Apply a convolution to two masks, then bitwise-and both masks

    @param mask_a (np.ndarray): First image mask
    @param mask_b (np.ndarray): Second image mask
    @param kernel_size (int): Size of kernel for convolution. Defaults to 5.
    @return np.ndarray: The bitwise-and of the masks after a convolution
    """
    kernel = np.ones((kernel_size, kernel_size), np.int8)
    mask_a_convolution = cv.filter2D(mask_a, -1, kernel)
    mask_b_convolution = cv.filter2D(mask_b, -1, kernel)
    and_convolutions = cv.bitwise_and(mask_a_convolution, mask_b_convolution)
    return and_convolutions


def gaussian_blur_discrete(img: np.ndarray, kernel_size: int = 5) -> np.ndarray:
    """
    Applies a Gaussian blur to an image

    @param img (np.ndarray): An OpenCV image
    @param kernel_size (int, optional): Size of kernel for Gaussian blur. Defaults to 5.
    @return np.ndarray: The image after the Gaussian blur
    """
    blur = cv.GaussianBlur(img, (kernel_size, kernel_size), 20)
    discrete_blur = cv.inRange(blur, 1, 255)
    return discrete_blur


def find_contours(img_mask: np.ndarray, epsilon=3, min_area=250) -> List[np.ndarray]:
    """
    Find rectangular contours in an image mask

    @param img_mask (np.ndarray): An image mask
    @param epsilon (int): Epsilon value for rectangular contour approximation. Defaults to 3.
    @param min_area (int): Minimum area of the contour region. Defaults to 250.
    @return List[np.ndarray]: List containing rectangular contours
    """
    contours, hierarchy = cv.findContours(
        img_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
    )

    rectangular_contours = []
    for contour in contours:
        approx = cv.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4:
            contour_area = cv.contourArea(approx)
            if contour_area > min_area:
                rectangular_contours.append(approx)

    return rectangular_contours


def detect_contours(img: np.ndarray) -> List[np.ndarray]:
    """
    Find contours of ArUco tags using masking

    This function was developed as an attempt to detect ArUco tags from long distances.
    It might be unnecessary once a rover search algorithm is implemented.

    @param img (np.ndarray): an OpenCV image
    @return List[np.ndarray]: List containing contours of ArUco tags
    """
    img_mask_black = mask_black(img)
    img_mask_white = mask_white(img)
    img_masks_combined = combine_masks(img_mask_black, img_mask_white)

    contours = find_contours(img_masks_combined)
    return contours


def estimate_distance_ft(corners: np.ndarray) -> float:
    """
    Estimate the distance to an ArUco tag in feet

    @param corners (np.ndarray): array containing the corners of the ArUco tag
    @return float: estimated distance to vision target in feet
    """
    side_lengths = [
        np.linalg.norm(corners[i - 1] - corners[i]) for i in range(len(corners))
    ]
    return SIDE_LENGTH_1FT / max(side_lengths)


def estimate_distance_m(corners: np.ndarray) -> float:
    """
    Estimate the distance to an ArUco tag in meters

    @param corners (np.ndarray): array containing the corners of the ArUco tag
    @return float: estimated distance to vision target in meters
    """
    return FT_TO_M * estimate_distance_ft(corners)


## @}
# @}
