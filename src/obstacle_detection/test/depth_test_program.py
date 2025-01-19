import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

pipe.start(cfg)

# Align color frame to depth frame
align = rs.align(rs.stream.depth)

first = True
while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipe.wait_for_frames()

    # Align the frames
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    # Convert frames to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply non-linear depth colormap for better visualization
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03),
        cv2.COLORMAP_JET
    )

    # Create point cloud
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)

    # Get vertices with color
    vertices = np.asanyarray(points.get_vertices())
    textures = np.asanyarray(points.get_texture_coordinates())

    # More advanced depth visualization
    depth_scaled = depth_image.astype(float)
    depth_normalized = cv2.normalize(
        depth_scaled,
        None,
        0,
        255,
        cv2.NORM_MINMAX
    )

    if first:
        print("Depth image shape:", depth_image.shape)
        print("Vertices shape:", vertices.shape)
        first = False

    # Display images
    cv2.imshow('RGB', color_image)
    cv2.imshow('Depth Colormap', depth_colormap)
    cv2.imshow('Depth Normalized', depth_normalized.astype(np.uint8))

    if cv2.waitKey(1) == ord('q'):
        break

pipe.stop()
cv2.destroyAllWindows()