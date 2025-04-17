import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd

pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

pipe.start(cfg)
prf = pipe.get_active_profile()

print("Streams in the profile:")
for stream in prf.get_streams():
    print(f"Stream format: {stream.format()}")
    print(f"Stream resolution: {stream.as_video_stream_profile().width()} x {stream.as_video_stream_profile().height()}")
    print("---")

# Get specific stream profiles
color_profile = prf.get_stream(rs.stream.color)
depth_profile = prf.get_stream(rs.stream.depth)

# Print detailed information about a specific stream
print("\nColor Stream Details:")
print(f"Width: {color_profile.as_video_stream_profile().width()}")
print(f"Height: {color_profile.as_video_stream_profile().height()}")
print(f"Format: {color_profile.format()}")
print(f"FPS: {color_profile.fps()}")

depth_sensor = prf.get_device().first_depth_sensor()
color_sensor = prf.get_device().first_color_sensor()

# Check emitter status
emitter_enabled = depth_sensor.get_option(rs.option.emitter_enabled)
print(f"Emitter Status: {'Enabled' if emitter_enabled else 'Disabled'}")

# You can also get the emitter's power level
emitter_power = depth_sensor.get_option(rs.option.laser_power)
print(f"Emitter Power Level: {emitter_power}")

power_range = depth_sensor.get_option_range(rs.option.laser_power)
print(f"Laser Power Range: {power_range.min} - {power_range.max}")

try:
    depth_sensor.set_option(rs.option.laser_power, 360)
    print("Laser power set to 100")
except Exception as e:
    print(f"Error setting laser power: {e}")

color_sensor.set_option(rs.option.auto_exposure_priority, 1)
print("Auto Exposure Priority enabled")

color_sensor.set_option(rs.option.enable_auto_exposure, 1)
print("Auto Exposure enabled")

# Check and set specific auto exposure parameters
try:
    # Disable manual exposure if needed
    color_sensor.set_option(rs.option.exposure, 0)  # 0 means auto

    # Optional: set auto exposure modes
    color_sensor.set_option(rs.option.exposure_mode, 1)  # typically 1 for auto
    print("Exposure mode set to auto")

    current_exposure = color_sensor.get_option(rs.option.exposure)
    print(f"Current Exposure: {current_exposure}")

except Exception as e:
    print(f"Error configuring auto exposure: {e}")

first = True
while True:
    frame = pipe.wait_for_frames()
    depth = frame.get_depth_frame()
    color = frame.get_color_frame()
    pc = rs.pointcloud()

    depth_image = np.asanyarray(depth.get_data())
    color_image = np.asanyarray(color.get_data())
    points = pc.calculate(depth)
    vertices = np.asanyarray(points.get_vertices())
    vertices = vertices.reshape(848, 480)
    column_names = [f'pixel_{x}' for x in range(vertices.shape[1])]
    df = pd.DataFrame(vertices, columns=column_names)
    if first:
        # print(depth_image.shape)
        # print(depth_image)
        # print(pc)
        print(depth_image.shape)
        #np.savetxt("foo.csv", vertices, delimiter=",")
        #df.to_csv('out.csv', index=False)

        first = False
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_JET)

    cv2.imshow('rgb', color_image)
    cv2.imshow('depth', depth_cm)

    if cv2.waitKey(1) == ord('q'):
        break

pipe.stop()

