import numpy as np
import cv2 as cv
import glob
import json

# termination criteria
termination_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

calibration_drawing_points = np.zeros((44, 3), np.float32) # 44 rows in checkboard and 3 columns for x, y, z coordinates of the point/circle that is being drawn
a = [0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 360] # I measured the X spacing between black squares horizontally across the checkerboard to be 36 mm so this list is the X coordinates of the points
b = [0, 72, 144, 216, 36, 108, 180, 252] # I measured the Y spacing between black and white squares vertically across the checkerboard to be 36 mm, so 72 mm between black squares, this list is the Y coordinates of the points
for i in range(0, 44):
    calibration_drawing_points[i] = (a[i // 4], (b[i % 8]), 0)

object_points = []
image_points = []

# Reading images from the folder, but we should use OpenCV VideoCapture to get readings every some seconds from our Webcam
"""
images = glob.glob('./images/*.jpg')  #TO FIX: comment out this code and instead run a loop and collect iamge frames from the OpenCV VideoCapture method to get realtime frames from our Webcam.
gray = None
i = 0
for f in images:
    img = cv.imread(f)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

"""
# Using OpenCV VideoCapture to get readings every some seconds from our Webcam
cap = cv.VideoCapture(0) # 0 is the default camera
gray = None 
i = 0  
while ret: #Loop through the frames until the frame is not read correctly
    ret, img = cap.read()  
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Converting the frame to grayscale

    # Just using a bunch of OpenCV function I got from a site I linked on the Sprint 3 doc
    ret, corners = cv.findCirclesGrid(gray, (4, 11), None, flags=cv.CALIB_CB_ASYMMETRIC_GRID) 

    if ret == True:
        object_points.append(calibration_drawing_points)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), termination_criteria)
        image_points.append(corners2)

        cv.drawChessboardCorners(img, (4, 11), corners2, ret) # Drawing the corners, saving and displaying the image
        cv.imwrite(f'./calibration_images/circle_drawings{i}.jpg', img)
        cv.imshow('img', img)
        cv.waitKey(0)
    i+=1
cv.destroyAllWindows()

ret, camera_mat, distortion, rotation_vecs, translation_vecs = cv.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None)



print("Error in projection : \n", ret)
print("Camera matrix : \n", camera_mat)
print("Camera focal length fx : \n", camera_mat[0,0])
print("Camera focal height fy : \n", camera_mat[1,1])
print("Camera optical center x coord cx : \n", camera_mat[0,2])
print("Camera optical center y coord cy : \n", camera_mat[1,2])
print("Distortion coefficients : \n", distortion)
print("Rotation vector : \n", rotation_vecs)
print("Translation vector : \n", translation_vecs)
calibration_data = {
    "projection_error": ret,
    "camera_matrix": camera_mat.tolist(),
    "focal_length_fx": camera_mat[0, 0],
    "focal_length_fy": camera_mat[1, 1],
    "optical_center_cx": camera_mat[0, 2],
    "optical_center_cy": camera_mat[1, 2],
    "distortion_coefficients": distortion.tolist(),
    "rotation_vectors": [vec.tolist() for vec in rotation_vecs],
    "translation_vectors": [vec.tolist() for vec in translation_vecs]
}

with open('camera_calibration_parameters.json', 'w') as json_file:
    json.dump(calibration_data, json_file, indent=4)
