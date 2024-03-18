#!/usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_and_display_lanes(image_path):
    def make_points(image, line):
        slope, intercept = line
        y1 = int(image.shape[0])
        y2 = int(y1 * 7 / 10)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return [(x1, y1, x2, y2)]

    def average_slope_intercept(image, lines):
        left_fit = []
        right_fit = []
        if lines is None or len(lines) == 0:
            return None
        for line in lines:
            for x1, y1, x2, y2 in line:
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))
        if not left_fit or not right_fit:
            return None
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = make_points(image, tuple(left_fit_average))
        right_line = make_points(image, tuple(right_fit_average))
        averaged_lines = [left_line, right_line]
        return averaged_lines

    def canny(img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        kernel = 5
        blur = cv2.GaussianBlur(gray, (kernel, kernel), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny

    def display_lines(img, lines):
        line_image = np.zeros_like(img)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return line_image

    def region_of_interest(canny):
        height = canny.shape[0]
        width = canny.shape[1]
        mask = np.zeros_like(canny)
        triangle = np.array([[
            (0, height),
            (246, 234),
            (560, height),
        ]], np.int32)
        cv2.fillPoly(mask, triangle, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image

    image = cv2.imread(image_path)
    lane_image = np.copy(image)
    canny_image = canny(lane_image)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = average_slope_intercept(lane_image, lines)
    line_image = display_lines(lane_image, averaged_lines)
    combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    cv2.imshow('test',combo_image)
    cv2.waitKey(0)

    return combo_image

# Example usage:
detect_and_display_lanes(r'/home/rufaid/Desktop/test.png')
