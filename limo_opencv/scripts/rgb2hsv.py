import cv2

# 读取图像
image = cv2.imread('../image.jpg')

# 转换为HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 显示结果
cv2.imshow('Original Image', image)
cv2.imshow('HSV Image', hsv_image)

# 等待按键
cv2.waitKey(0)
cv2.destroyAllWindows()
