import cv2
import numpy as np
import json

# Load the JSON data
with open('data.json', 'r') as f:
    calibration_data = json.load(f)

# Extract camera matrix and distortion coefficients
camera_matrix = np.array(calibration_data['camera_matrix'])
dist_coeffs = np.array(calibration_data['distortion_coefficients'])

# Load the image
imagepath = 'tag.png'
img = cv2.imread(imagepath)

# Convert the image to grayscale
img = img.mean(axis=2).astype(np.uint8)
print(img.shape)

# Detect AprilTags
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

detections = at_detector.detect(img)
print(detections)

# Hardcoded 2D image points (detected corners of the AprilTag in the image)
image_points = np.array([
    [257.42001343, 264.6784668],  # Detected bottom-left corner in image
    [347.35012817, 264.63848877],  # Detected bottom-right corner in image
    [345.87304687, 181.70478821],  # Detected top-right corner in image
    [259.81695557, 181.46292114]   # Detected top-left corner in image
])

# Define 3D model points (4 corners of the AprilTag in its own coordinate frame, size 122mm x 122mm)
tag_size = 0.122  # AprilTag size in meters
half_size = tag_size / 2.0

model_points = np.array([
    [-half_size, -half_size, 0],   # Bottom-left corner
    [half_size, -half_size, 0],    # Bottom-right corner
    [half_size, half_size, 0],     # Top-right corner
    [-half_size, half_size, 0]     # Top-left corner
])

# Call solvePnP to get the rotation and translation vectors
success, rvec, tvec = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)

# Optional - Convert rotation vector to rotation matrix
rotation_matrix, _ = cv2.Rodrigues(rvec)

# Display or use the results (e.g., rvec and tvec give pose relative to camera)
print("Rotation Vector:\n", rvec)
print("Translation Vector:\n", tvec)
print("Rotation Matrix:\n", rotation_matrix)