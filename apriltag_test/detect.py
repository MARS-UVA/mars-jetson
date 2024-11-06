import cv2
import numpy as np

imagepath = 'tag.png'
img = cv2.imread(imagepath)

img = img.mean(axis=2).astype(np.uint8)
print(img.shape)
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

print(at_detector.detect(img))