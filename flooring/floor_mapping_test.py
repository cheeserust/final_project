import cv2
import cv2.aruco as aruco
import yaml
import os

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(PACKAGE_DIR, 'config', 'floor_markers.yaml')) as f:
    floor_map = yaml.safe_load(f)['floor_markers']

print("Loaded floor mapping:")
for marker_id, info in floor_map.items():
    print(f"  ID {marker_id} -> floor {info['floor_number']} ({info['map_yaml']})")
print()

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, params)

cap = cv2.VideoCapture("/dev/video0") # Change camera index if needed
if not cap.isOpened():
    print("Camera failed to open")
    exit()

last_reported = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(frame)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for marker_id in ids.flatten():
            if marker_id in floor_map:
                floor_info = floor_map[marker_id]
                if marker_id != last_reported:
                    print(f"MATCH: marker {marker_id} -> floor {floor_info['floor_number']}, "
                          f"map={floor_info['map_yaml']}")
                    last_reported = marker_id
            else:
                if marker_id != last_reported:
                    print(f"UNMAPPED: marker {marker_id} detected but not in floor_markers.yaml")
                    last_reported = marker_id

    cv2.imshow("floor mapping check", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()