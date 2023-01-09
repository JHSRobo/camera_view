import cv2

camera_ip = input("Ending IP Number? (Just the last octet. If it is 192.168.1.2, type 2) >> ")

cap = cv2.VideoCapture("http://192.168.1." + str(camera_ip) + ":5000/")
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
while cap.isOpened():
    ret, frame = cap.read()
    if frame is None:
        continue
    cv2.imshow("Camera Feed", frame)
    cv2.waitKey(1)
