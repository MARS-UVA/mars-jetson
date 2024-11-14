import cv2 as cv
import numpy as np
import math
import detect

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

tag_size = 122 # 122 mm
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
    [672.80742581, 0., 302.72097471],
    [0., 671.97164633, 246.43218186],
    [0., 0., 1.]
])

dist = np.array([0.10255418, -1.05133916, 0.00477888, -0.0063217, 2.40837452])


ret,rvecs, tvecs = cv.solvePnP(objp, corners, mtx, dist)


axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
# imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
# img = np.zeros((480, 640, 3), dtype=np.uint8)
# img = draw(img,corners,imgpts)
# cv.imshow('img',img)

#cv.destroyAllWindows()

distance = np.linalg.norm(tvecs)
rotation_matrix, _ = cv.Rodrigues(rvecs)

print("Rotation Vector:\n", rvecs)
print("Rotation Matrix:\n", rotation_matrix)
print("Translation Vector:\n", tvecs)
print("Distance:\n", distance)

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

# Convert the angles from radians to degrees
x_angle = math.degrees(x_angle)
y_angle = math.degrees(y_angle)
z_angle = math.degrees(z_angle)

print("Roll (X):", x_angle)
print("Pitch (Y):", y_angle)
print("Yaw (Z):", z_angle)