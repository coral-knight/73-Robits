import cv2
import numpy as np
from PIL import Image

c = [59, 123, 134]
hsv_color = cv2.cvtColor(np.array([[[c[2], c[1], c[0]]]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0,0]
print(hsv_color)