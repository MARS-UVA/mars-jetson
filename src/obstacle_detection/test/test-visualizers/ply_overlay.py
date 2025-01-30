import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from plyfile import PlyData
import open3d as o3d

sns.set_theme(style="darkgrid")
plt.rcParams['figure.figsize'] = [10, 8]

print("Which PLY viewer to use: Matplotlib (1) or Open3D (2)?")
viewer_in = int(input())

print("First PLY file path?")
ply_path1 = input()

print("Second PLY file path?")
ply_path2 = input()

if viewer_in == 1:
    print("2D or 3D representation (2 for 2D or 3 for 3D)?")
    viewer_in = int(input())
    
    plydata1 = PlyData.read(ply_path1)
    plydata2 = PlyData.read(ply_path2)

    vertices1 = plydata1['vertex'].data
    vertices2 = plydata2['vertex'].data

    x1 = vertices1['x']
    y1 = vertices1['y']
    z1 = vertices1['z']

    x2 = vertices2['x']
    y2 = vertices2['y']
    z2 = vertices2['z']

    if viewer_in == 2:
        plt.figure()
        
        scatter1 = plt.scatter(x1, y1, c=z1, cmap='viridis', s=1, alpha=0.6, 
                             label='Input Point Cloud')
        
        scatter2 = plt.scatter(x2, y2, c='red', s=10, alpha=0.8, 
                             label='Classified Obstacles')
        
        cbar = plt.colorbar(scatter1)
        cbar.set_label('Height from ground (Input Point Cloud)', fontsize=10)
        
        # Customize plot appearance
        plt.title('Overlaid Point Clouds 2D Projection', fontsize=12, pad=15)
        plt.xlabel('X', fontsize=10)
        plt.ylabel('Y', fontsize=10)

        plt.legend(frameon=True, fancybox=True, framealpha=0.9, fontsize=10)

        sns.despine(left=False, bottom=False)
        
        plt.tight_layout()
        plt.show()
        
    elif viewer_in == 3:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        scatter1 = ax.scatter(x1, y1, z1, c=z1, cmap='viridis', s=1, alpha=0.6,
                            label='Input Point Cloud')
        scatter2 = ax.scatter(x2, y2, z2, c='red', s=1, alpha=0.8,
                            label='Classified Obstacles')
        
        cbar = plt.colorbar(scatter1)
        cbar.set_label('Height from ground (Input Point Cloud)', fontsize=10)
        
        ax.set_title('Overlaid Point Clouds Visualization', fontsize=12, pad=15)
        ax.set_xlabel('X', fontsize=10)
        ax.set_ylabel('Y', fontsize=10)
        ax.set_zlabel('Z', fontsize=10)
        
        ax.set_facecolor('#eaeaf2')
        fig.patch.set_facecolor('white')
        
        ax.legend(frameon=True, fancybox=True, framealpha=0.9, fontsize=10)
        
        plt.tight_layout()
        plt.show()
        
elif viewer_in == 2:
    pcd1 = o3d.io.read_point_cloud(ply_path1)
    pcd2 = o3d.io.read_point_cloud(ply_path2)

    pcd1.normalize_normals()
    pcd1.paint_uniform_color([1, 0.706, 0])
    pcd2.paint_uniform_color([1, 0, 0])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    vis.run()
    vis.destroy_window()
    
else:
    print("Invalid input. Please enter 1 or 2.")