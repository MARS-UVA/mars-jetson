import cv2 as cv
import numpy as np
import math
import detect
from pupil_apriltags import Detector

at_detector = Detector(
      families="tagStandard41h12",
      nthreads=1,
      quad_decimate=1.0,
      quad_sigma=0.0,
      refine_edges=1,
      decode_sharpening=0.25,
      debug=0
   )

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    corner = tuple(int(x) for x in corner)
    img = cv.line(img, corner, tuple(int(x) for x in tuple(imgpts[0].ravel())), (255,0,0), 5)
    img = cv.line(img, corner, tuple(int(x) for x in tuple(imgpts[1].ravel())), (0,255,0), 5)
    img = cv.line(img, corner, tuple(int(x) for x in tuple(imgpts[2].ravel())), (0,0,255), 5)
    return img

tag_size = 66.675 # 122 mm
objp = np.array([
    [0, 0, 0],
    [tag_size, 0, 0],
    [tag_size, tag_size, 0],
    [0, tag_size, 0]
], dtype=np.float32)

corners = np.array([
    [299.07305908, 257.19723511],
    [369.17623901, 261.88638306],
    [369.79840088, 202.53692627],
    [303.2673645, 200.62452698]
], dtype=np.float32)

#corners = detect.detect()

corners = corners.reshape(-1, 1, 2)

mtx = np.array([
    [1329.143348, 0., 945.392392],
    [0., 1326.537785, 521.144703],
    [0., 0., 1.]
])

dist = np.array([-0.348650, 0.098710, -0.000157, -0.001851, 0.000000])


axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

cap = cv.VideoCapture(1)
while(True):
    ret, frame = cap.read()
    cv.imshow('frame', frame)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    det = at_detector.detect(gray)
    if not det:
        continue
    corners = det[0].corners
    if len(corners) > 0:
        corner = corners.astype(np.float32)
        ret, rvecs, tvecs = cv.solvePnP(objp, corner, mtx, dist)
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)

        distance = np.linalg.norm(tvecs)
        rotation_matrix, _ = cv.Rodrigues(rvecs)

        scale_factor = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
        singular = scale_factor < 1e-6

        if not singular:
            x_angle = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])  # Roll
            y_angle = math.atan2(-rotation_matrix[2, 0], scale_factor)                    # Pitch
            z_angle = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  # Yaw
        else:
            x_angle = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y_angle = math.atan2(-rotation_matrix[2, 0], scale_factor)
            z_angle = 0

        x_angle = math.degrees(x_angle)
        y_angle = math.degrees(y_angle)
        z_angle = math.degrees(z_angle)

        print("Roll (X):", x_angle)
        print("Pitch (Y):", y_angle)
        print("Yaw (Z):", z_angle)

        # print("Rotation Vector:\n", rvecs)
        print("Rotation Matrix:\n", rotation_matrix.T)
        # print("Translation Vector:\n", tvecs)
        print("Distance:\n", distance)
        #frame = draw(frame, corner, imgpts)


    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cv.destroyAllWindows()