import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData
1
print("Which PLY viewer to use: Matplotlib (1) or Open3D (2)?")
viewer_in = int(input())

if viewer_in == 1:
    plydata = PlyData.read('out.ply')

    vertices = plydata['vertex'].data

    x = vertices['x']
    y = vertices['y']
    z = vertices['z']

    theta = np.pi/2
    y_rot = y*np.cos(theta) - z*np.sin(theta)
    z_rot = y*np.sin(theta) + z*np.cos(theta)

    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(x, y_rot, z_rot, c=z, cmap='viridis', s=1)
    plt.colorbar(scatter)
    plt.title('Point Cloud Visualization')
    plt.show()
elif viewer_in == 2:
    import open3d as o3d

    pcd = o3d.io.read_point_cloud("out.ply")

    o3d.visualization.draw_geometries([pcd])

    pcd.normalize_normals()
    pcd.paint_uniform_color([1, 0.706, 0])  # set color

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
else:
    print("Invalid input. Please enter 1 or 2.")