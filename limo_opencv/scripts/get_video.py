import cv2

# 打开视频文件
cap = cv2.VideoCapture('../LIMO.mp4')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    resized_frame = cv2.resize(frame, (1000, 600))
    cv2.imshow('Frame', resized_frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
