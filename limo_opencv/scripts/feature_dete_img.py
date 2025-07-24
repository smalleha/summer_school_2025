# 导入OpenCV库
import cv2
import numpy as np

# 读取图像
image01 = cv2.imread('../image.jpg')
image = cv2.imread('../image01.png')

# Harris 角点检测
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray01 = cv2.cvtColor(image01, cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2, 3, 0.1)
# 标记角点
image[dst > 0.01 * dst.max()] = [0, 0, 255]

# ORB 特征点检测
orb = cv2.ORB_create()
keypoints, descriptors = orb.detectAndCompute(gray01, None)
# 绘制特征点
image01 = cv2.drawKeypoints(image01, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# 显示图像
cv2.imshow('Image', image)
cv2.imshow('Image01', image01)
cv2.waitKey(0)
cv2.destroyAllWindows()
