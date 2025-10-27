import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Vertex:
    x: float
    y: float
    z: float

# Set Seaborn style
sns.set_theme(style="darkgrid")
plt.rcParams['figure.figsize'] = [12, 8]

def capture_depth_points() -> Tuple[List[Vertex], List[Vertex]]:
    # Initialize RealSense pipeline
    pipe = rs.pipeline()
    cfg = rs.config()
    
    # Configure streams
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    
    # Start pipeline
    profile = pipe.start(cfg)
    
    # Get depth sensor's intrinsics
    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    depth_intrinsics = depth_profile.get_intrinsics()
    
    # Warm up camera
    print("Warming up camera...")
    for _ in range(30):
        pipe.wait_for_frames()
    
    # Create decimation filter
    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 4)
    
    # Get frames
    frames = pipe.wait_for_frames()
    depth_frame = frames.get_depth_frame()  # Get depth frame
    decimated_depth = decimation.process(depth_frame)  # Apply decimation
    
    # Lists to store vertices
    vertices_original = []
    vertices_rotated = []
    
    # Rotation angle (45 degrees)
    REALSENSE_ANGLE = -45.0  # degrees
    angle_rad = (REALSENSE_ANGLE * np.pi) / 180.0
    
    # Get dimensions from the depth frame
    height = decimated_depth.get_height()
    width = decimated_depth.get_width()
    
    print(f"Processing frame of size {width}x{height}")
    
    # Process depth data
    for y in range(height):
        for x in range(width):
            depth = decimated_depth.get_distance(x, y)
            
            if depth > 0:
                # Deproject from pixel to 3D point
                point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
                
                # Store original point
                vertices_original.append(Vertex(point[0], point[1], point[2]))
                
                # Calculate rotated coordinates
                y_rotated = point[1] * np.cos(angle_rad) - point[2] * np.sin(angle_rad)
                z_rotated = point[1] * np.sin(angle_rad) + point[2] * np.cos(angle_rad)
                
                # Store rotated point
                vertices_rotated.append(Vertex(point[0], y_rotated, z_rotated))
    
    pipe.stop()
    return vertices_original, vertices_rotated

def plot_vertices(vertices_original: List[Vertex], vertices_rotated: List[Vertex]):
    # Extract coordinates
    x_orig = [v.x for v in vertices_original]
    y_orig = [v.y for v in vertices_original]
    z_orig = [v.z for v in vertices_original]
    
    x_rot = [v.x for v in vertices_rotated]
    y_rot = [v.y for v in vertices_rotated]
    z_rot = [v.z for v in vertices_rotated]
    
    # Create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot original points with low opacity
    scatter_orig = ax.scatter(x_orig, y_orig, z_orig, 
                            c=z_orig, cmap='viridis', s=1, alpha=0.2,
                            label='Original Points')
    
    # Plot rotated points
    scatter_rot = ax.scatter(x_rot, y_rot, z_rot, 
                           c=z_rot, cmap='plasma', s=1, alpha=0.8,
                           label='Rotated Points')
    
    # Customize plot
    ax.set_title('Point Cloud Before and After Rotation', fontsize=12, pad=15)
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    
    # Add colorbars
    plt.colorbar(scatter_orig, label='Original Depth (m)', pad=0.1)
    plt.colorbar(scatter_rot, label='Rotated Depth (m)', pad=0.15)
    
    # Add legend
    ax.legend(frameon=True, fancybox=True, framealpha=0.9, fontsize=10)
    
    plt.tight_layout()
    plt.show()

def main():
    print("Capturing depth data...")
    vertices_original, vertices_rotated = capture_depth_points()
    
    print(f"Captured {len(vertices_original)} points")
    print("Plotting points...")
    plot_vertices(vertices_original, vertices_rotated)

if __name__ == "__main__":
    main()