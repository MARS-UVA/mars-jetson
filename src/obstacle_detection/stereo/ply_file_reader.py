import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData
import open3d as o3d

print("Which PLY viewer to use: Matplotlib (1) or Open3D (2)?")
viewer_in = int(input())

print("PLY file path?")
ply_path = input()

if viewer_in == 1:
    plydata = PlyData.read(ply_path)

    vertices = plydata['vertex'].data

    x = vertices['x']
    y = vertices['y']
    z = vertices['z']

    # theta = (-5 * np.pi) / 180
    # y_rot = y*np.cos(theta) - z*np.sin(theta)
    # z_rot = y*np.sin(theta) + z*np.cos(theta)

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(x, y, z, c=z, cmap='viridis', s=1)
    plt.colorbar(scatter)
    plt.title('Point Cloud Visualization')
    plt.show()
elif viewer_in == 2:
    pcd = o3d.io.read_point_cloud(ply_path)

    o3d.visualization.draw_geometries([pcd])

    pcd.normalize_normals()
    pcd.paint_uniform_color([1, 0.706, 0])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
else:
    print("Invalid input. Please enter 1 or 2.")