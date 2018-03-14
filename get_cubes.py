import numpy as np

import cv2


class CubeTracker(object):
    def __init__(self):
        pass

    def find_bounding_boxes(self, img, display=True):
        # Get bounding boxes around red and green rectangles
        image = cv2.GaussianBlur(img, (5, 5), 3)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        edges = cv2.Canny(image, 60, 60)
        binary_image = self.get_bounding_boxes(hsv_image)

        im, contours, hierarchy = cv2.findContours(edges, 1, 2)
        cnt = contours[0]
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

        if display:
            cv2.imshow("Binary Image With Morphology", binary_image)
            cv2.imshow("Original images", image)
            cv2.imshow("HSV image", hsv_image)
            cv2.imshow('edge', edges)
            cv2.waitKey(0)
        return binary_image

    def get_bounding_boxes(self, hsv_image):
        binary_image1 = cv2.inRange(hsv_image, np.array([10, 60, 60], dtype="uint8"),
                                    np.array([30, 255, 255], dtype="uint8"))
        return cv2.bitwise_or(binary_image1, binary_image1)


if __name__ == '__main__':
    tracker = CubeTracker()
    binary_image = tracker.find_bounding_boxes(cv2.imread("2.jpg"), True)
