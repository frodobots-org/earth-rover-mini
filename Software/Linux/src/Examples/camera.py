import cv2
'''
rtsp://<Earth Rover Mini IP>/live/0  Front camera main-stream
rtsp://<Earth Rover Mini IP>/live/1  Front camera sub-stream
rtsp://<Earth Rover Mini IP>/live/2  Rear camera main-stream
rtsp://<Earth Rover Mini IP>/live/3  Rear camera sub-stream
'''
cap = cv2.VideoCapture("rtsp://192.168.11.1/live/0")
if not cap.isOpened():
    print("Failed to open RTSP stream")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    cv2.imshow("RTSP Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()