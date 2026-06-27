# aruco_test.py

import cv2
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, params)

cap = cv2.VideoCapture(5)
if not cap.isOpened():
    print("Camera failed to open — try index 1, 2, etc.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    corners, ids, rejected = detector.detectMarkers(frame)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        print("Detected IDs:", ids.flatten())

    cv2.imshow("aruco test", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()