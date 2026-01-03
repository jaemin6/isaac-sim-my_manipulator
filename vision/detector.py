# vision/detector.py
import cv2
import numpy as np

class SimpleObjectDetector:
    def __init__(
        self,
        hsv_lower=(0, 120, 70),
        hsv_upper=(10, 255, 255),
        min_area=500,
    ):
        """인식할 대상의 색상 범위(HSV)와 최소 크기 설정"""
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        self.min_area = min_area

    def detect(self, rgb_img):
        """
        입력된 RGB 이미지에서 물체를 감지하여 위치 정보를 반환
        Returns:
            center: (x, y) 중심점
            bbox: (x, y, w, h) 바운딩 박스
            mask: 이진화 처리된 마스크 이미지
        """
        # 1. 색상 공간 변환 (RGB -> HSV)
        
        # 2. 마스크 생성 및 노이즈 제거
        
        # 3. 윤곽선(Contour) 검출 및 가장 큰 물체 찾기
        
        # 4. 결과값 계산 및 반환
        return None, None, None