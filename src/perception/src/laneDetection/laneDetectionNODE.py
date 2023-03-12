#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image
from perception.msg import lineArray, line

import cv2 as cv

"""
    Variables declarations
"""
kernel_size = 3
match_mask_color = 255 # value of lines' color

# define range of black color in HSV color space

# define Canny edge detection parameters
low_threshold = 200
high_threshold = 400

# Hough transform parameters
rho = 1             # distance precision in pixel
angle = np.pi / 180 # angular precision in radian
min_threshold = 10  # min number of votes for valid line

# Rectangular Kernel
rectangular_kernel = cv.getStructuringElement(cv.MORPH_RECT, (7,7))


"""
    Functions definitions and declarations
"""

def preprocess_image(image):
    # Convert image to grayscale
    gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # remove noise with gaussian blur
    denoised_image = cv.GaussianBlur(gray_image, (kernel_size, kernel_size), 0)
    # dilate the denoised image
    dilated_image = cv.dilate(denoised_image, rectangular_kernel, iterations=2)

    return dilated_image


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # define the vertices of the trapezoid
    trap_top_width = 1/3 # width of the top of the trapezoid (in fraction of the frame width)
    trap_bottom_width = 2/3 # width of the bottom of the trapezoid (in fraction of the frame width)
    trap_height = height * 1/2 # height of the trapezoid (in fraction of the frame height)
    trap_top_xoffset = (1 - trap_top_width) / 2 # x-offset of the top of the trapezoid (in fraction of the frame width)
    # vertices = np.array([[
    #     ((width * trap_top_xoffset), height * (1 - trap_height)),
    #     ((width * (1 - trap_top_xoffset)), height * (1 - trap_height)),
    #     ((width * (1 - trap_bottom_width) / 2), height * 0.9),
    #     ((width * (1 - (1 - trap_bottom_width) / 2)), height * 0.9),
    # ]], dtype=np.int32)
    vertices = np.array([[
        (width * 0.4, height * 0.425),
        (width * 0.65, height * 0.425),
        (width * 0.95, height * 0.9),
        (width * 0.05, height * 0.9),
    ]], dtype=np.int32)

    # only focus on the trapezoid region
    mask = np.zeros_like(edges)
    cv.fillPoly(mask, vertices, 255)
    masked_edges = cv.bitwise_and(edges, mask) 

    return masked_edges


def detect_segments(cropped_edges_image):
    line_segments = cv.HoughLinesP(cropped_edges_image, rho, angle,
                                    min_threshold, np.array([]),
                                    minLineLength=20, # min allowed length of a single line
                                    maxLineGap=15 # max allowed gap between line for joining them together
                                    )
    return line_segments


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height # bottom of the frame
    y2 = int(y1 * .5) # make points from bottom of the frame up

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    lane_lines = []
    lines_kept = [] # DO NOT REMOVE - DEBUGGING
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit, right_fit = [], []

    boundary_left = 0.35
    boundary_right = 0.3
    left_region_boundary = width * (1 - boundary_left) 
    right_region_boundary = width * boundary_right

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2: # slope = inf
                # TODO logging message
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope, intercept = fit[0], fit[1]

            if abs(slope) < 0.1: # skip line segments with slopes close to  - horizontal lines
                continue

            lines_kept.append(line_segment)
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))


    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines, lines_kept


def draw_lines(image, lines, line_color=(0, 255, 255), line_width=10):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv.line(line_image, (x1, y1), (x2, y2), line_color, 1, 1)

    new_image = cv.addWeighted(image, 0.8, line_image, 1, 1)
    return new_image

def detect_crosswalk(image):
    height, width = image.shape

    vertices = np.array([[
        (0, height * 0.4),
        (width, height * 0.4),
        (width, height * 0.55),
        (0, height * 0.55),
    ]], dtype=np.int32)

    mask = np.zeros_like(image)
    cv.fillPoly(mask, vertices, 255)
    roi = cv.bitwise_and(image, mask)

    # cv.imshow('Crosswalk', roi)
    # if cv.waitKey(20) == 27:
    #     sys.exit(0)

    contours, hierarchy = cv.findContours(roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    crosswalk = []
    for cnt in contours:
        x,y,w,h = cv.boundingRect(cnt)
        area = cv.contourArea(cnt)
        # ratio = h / w
        if area < 400: # or ratio < 1:
            continue
        crosswalk.append(cnt)

    return len(crosswalk) > 0

class laneDetectionNODE():
    def __init__(self):
        # TODO docstring
        rospy.init_node('laneDetectionNODE', anonymous=False)
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)
        self.lane_publisher = rospy.Publisher("/lane_info", lineArray, queue_size=1)

        cv.startWindowThread()
        cv.namedWindow("Test")
        cv.startWindowThread()
        # cv.namedWindow("Crosswalk")

    def run(self):
        rospy.loginfo('starting laneDetectionNODE')
        rospy.spin()    


    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data.
                        dtype=dtype, buffer=img_msg.data) # Since OpenCV works with BGR natively, we don't need to reorder thechannels.
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        return image_opencv


    def _streams(self, msg):
        image = self.imgmsg_to_cv2(msg)
        h, w, _ = image.shape

        # preprocess image
        preprocessed_image = preprocess_image(image)
        # Canny edge detection
        edges_image = cv.Canny(preprocessed_image, low_threshold, high_threshold)
        is_crosswalk = detect_crosswalk(edges_image)

        # extract the region of interest
        roi_image = region_of_interest(edges_image)

        # HoughLinesP method to directly obtain line end points
        lines_detected = detect_segments(roi_image)
        lane_lines, lines_kept = average_slope_intercept(image, lines_detected)

        lines_detected_image = None
        if lines_kept is not []:
            lines_detected_image = draw_lines(image, lines_kept)
        lines_image = None
        if lane_lines is not []:
            lines_image = draw_lines(image, lane_lines)

        message = lineArray()
        # Iterate over points coordinates
        for points in lane_lines:
            x1, y1, x2, y2 = points[0]
            # Draw the lines joining the points
            cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            ln = line()
            ln.x1 = x1
            ln.y1 = y1
            ln.x2 = x2
            ln.y2 = y2
            message.lines.append(ln)
            message.is_crosswalk = 1 if is_crosswalk else 0
        
        self.lane_publisher.publish(message)

        if lane_lines is not []:
            cv.imshow('Test', lines_image)
            if cv.waitKey(20) == 27:
                sys.exit(0)


if __name__ == "__main__":
    laneDetection = laneDetectionNODE()
    laneDetection.run()
