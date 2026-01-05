# vision/detector.py
import cv2
import numpy as np


class SimpleObjectDetector:
    def __init__(
        self,
        hsv_lower=(0, 50, 50),
        hsv_upper=(10, 255, 255),
        min_area=100,
    ):
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        self.min_area = min_area

    def detect(self, rgb_img):
        """
        입력: RGB 이미지 (H, W, 3)
        출력:
          - center (x, y) or None
          - bbox (x, y, w, h) or None
          - mask
        """

        # OpenCV는 BGR
        bgr = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, None, mask

        # 가장 큰 물체
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_area:
            return None, None, mask

        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        cy = y + h // 2

        return (cx, cy), (x, y, w, h), mask
