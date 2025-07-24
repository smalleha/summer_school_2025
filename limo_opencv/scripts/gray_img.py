# 导入OpenCV库
import cv2

# 读取图像
image = cv2.imread('../image.jpg')

# 将图像转换为灰度图像
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 保存图像
cv2.imwrite('output.jpg', gray_image)

# 显示图像
cv2.imshow('Image', image)
cv2.imshow('Gray_Image', gray_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
