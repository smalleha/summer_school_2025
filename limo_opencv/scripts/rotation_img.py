# 导入OpenCV库
import cv2
# 读取图像
image = cv2.imread('../image.jpg')
# 缩放图像
resized_image = cv2.resize(image, (400, 400))

# 获取图像的尺寸
(h, w) = resized_image.shape[:2]
# 计算旋转中心
center = (w // 2, h // 2)
# 定义旋转矩阵
M = cv2.getRotationMatrix2D(center, 45, 1)
# 旋转图像
rotated_image = cv2.warpAffine(resized_image, M, (w, h))

# 显示图像
cv2.imshow('Image', image)
cv2.imshow('rotated_image', rotated_image)
cv2.imshow('resized_image', resized_image)

cv2.waitKey(0)
cv2.destroyAllWindows()
