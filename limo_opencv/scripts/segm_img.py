import cv2

# 读取图像
image = cv2.imread('../image.jpg')

# 转换为灰度图像
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 二值化处理
ret, thresh = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

# 显示结果
cv2.imshow('Original Image', image)
cv2.imshow('Gray Image', gray_image)
cv2.imshow('Thresholded Image', thresh)

# 等待按键
cv2.waitKey(0)
cv2.destroyAllWindows()
