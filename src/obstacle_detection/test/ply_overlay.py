import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData
import open3d as o3d

print("Which PLY viewer to use: Matplotlib (1) or Open3D (2)?")
viewer_in = int(input())

print("First PLY file path?")
ply_path1 = input()

print("Second PLY file path?")
ply_path2 = input()

if viewer_in == 1:
    print("2D or 3D representation (2 for 2D or 3 for 3D)?")
    viewer_in = int(input())
    
    # Read both PLY files
    plydata1 = PlyData.read(ply_path1)
    plydata2 = PlyData.read(ply_path2)

    # Extract vertices from both files
    vertices1 = plydata1['vertex'].data
    vertices2 = plydata2['vertex'].data

    x1 = vertices1['x']
    y1 = vertices1['y']
    z1 = vertices1['z']

    x2 = vertices2['x']
    y2 = vertices2['y']
    z2 = vertices2['z']

    if viewer_in == 2:
        plt.figure(figsize=(10,8))
        # Plot first point cloud with height-based coloring
        scatter1 = plt.scatter(x1, y1, c=z1, cmap='viridis', s=1, label='First Point Cloud')
        # Plot second point cloud in red
        scatter2 = plt.scatter(x2, y2, c='red', s=15, label='Second Point Cloud')
        
        plt.colorbar(scatter1, label='Height from ground (First Point Cloud)')
        plt.title('Overlaid Point Clouds 2D Projection')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.show()
        
    elif viewer_in == 3:
        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')
        # Plot first point cloud with height-based coloring
        scatter1 = ax.scatter(x1, y1, z1, c=z1, cmap='viridis', s=1, label='First Point Cloud')
        # Plot second point cloud in red
        scatter2 = ax.scatter(x2, y2, z2, c='red', s=1, label='Second Point Cloud')
        
        plt.colorbar(scatter1, label='Height from ground (First Point Cloud)')
        plt.title('Overlaid Point Clouds Visualization')
        ax.legend()
        plt.show()
        
elif viewer_in == 2:
    # Read both point clouds
    pcd1 = o3d.io.read_point_cloud(ply_path1)
    pcd2 = o3d.io.read_point_cloud(ply_path2)

    # Set colors for the point clouds
    pcd1.normalize_normals()
    pcd1.paint_uniform_color([1, 0.706, 0])  # Original yellow color
    pcd2.paint_uniform_color([1, 0, 0])      # Red color for second point cloud

    # Visualize both point clouds together
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    vis.run()
    vis.destroy_window()
    
else:
    print("Invalid input. Please enter 1 or 2.")