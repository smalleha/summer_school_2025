# 导入OpenCV库
import cv2

# 读取图像
image = cv2.imread('../image.jpg')

# Canny 边缘检测
edges = cv2.Canny(image, 0, 200)

# 高斯模糊
blurred_image = cv2.GaussianBlur(image, (15, 15), 0)

# 显示图像
cv2.imshow('Image', image)
cv2.imshow('edges', edges)
cv2.imshow('blurred_image', blurred_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
