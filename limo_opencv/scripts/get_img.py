import cv2

# 打开摄像头
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

while True:
    ret, frame = cap.read()
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
