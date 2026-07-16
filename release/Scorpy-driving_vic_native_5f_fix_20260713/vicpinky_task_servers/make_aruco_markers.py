import cv2
import os

out_dir = "aruco_markers"
os.makedirs(out_dir, exist_ok=True)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

marker_id = 20

img = cv2.aruco.generateImageMarker(
    dictionary,
    marker_id,
    1000
)

path = os.path.join(
    out_dir,
    f"aruco_id_{marker_id}.png"
)

cv2.imwrite(path, img)
print(f"saved : {path}")
