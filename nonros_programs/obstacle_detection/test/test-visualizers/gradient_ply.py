import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from plyfile import PlyData
import open3d as o3d

# Set Seaborn style
sns.set_theme(style="darkgrid")
plt.rcParams['figure.figsize'] = [12, 8]

print("Which PLY viewer to use: Seaborn (1) or Open3D (2)?")
viewer_in = int(input())

print("PLY file path?")
ply_path = input()

if viewer_in == 1:
    print("2D or 3D representation (2 for 2D or 3 for 3D)?")
    viewer_in = int(input())
    
    print("Enter sampling rate (1 = all points, 2 = every second point, etc.):")
    sampling_rate = int(input())
    
    plydata = PlyData.read(ply_path)
    vertices = plydata['vertex'].data

    # Sample the points for better performance
    x = vertices['x'][::sampling_rate]
    y = vertices['y'][::sampling_rate]
    z = vertices['z'][::sampling_rate]

    if viewer_in == 2:
        # Create 2D visualization with lines
        plt.figure()
        
        # Create scatter plot with Seaborn
        scatter = plt.scatter(x, y, c=z, cmap='viridis', s=1, alpha=0.6)
        
        # Add colorbar with Seaborn styling
        cbar = plt.colorbar(scatter)
        cbar.set_label('Height from ground', fontsize=12)
        
        # Customize plot appearance
        plt.title('Point Cloud 2D Projection', fontsize=14, pad=20)
        plt.xlabel('X', fontsize=12)
        plt.ylabel('Y', fontsize=12)
        
        # Add a despine effect
        sns.despine(left=False, bottom=False)
        
        # Adjust layout
        plt.tight_layout()
        plt.show()
        
    elif viewer_in == 3:
        # Create 3D visualization with lines from ground
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Create scatter plot for points
        scatter = ax.scatter(x, y, z, c=z, cmap='viridis', s=1, alpha=0.8)
        
        # Draw lines from ground to each point with increased line weight
        for i in range(len(x)):
            ax.plot([x[i], x[i]], [y[i], y[i]], [0, z[i]], 
                   color='gray', alpha=0.2, linewidth=2.0)  # Increased linewidth from 0.5 to 2.0
        
        # Add colorbar with Seaborn styling
        cbar = plt.colorbar(scatter)
        cbar.set_label('Height', fontsize=12)
        
        # Customize plot appearance
        ax.set_title('Point Cloud Visualization with Ground Lines', fontsize=14, pad=20)
        ax.set_xlabel('X', fontsize=12)
        ax.set_ylabel('Y', fontsize=12)
        ax.set_zlabel('Z', fontsize=12)
        
        # Set background color to match Seaborn style
        ax.set_facecolor('#eaeaf2')
        fig.patch.set_facecolor('white')
        
        # Adjust layout
        plt.tight_layout()
        plt.show()

elif viewer_in == 2:
    # Read point cloud
    pcd = o3d.io.read_point_cloud(ply_path)
    points = np.asarray(pcd.points)
    
    # Create lines from ground to points
    lines = []
    line_colors = []
    
    # Sample points for better performance
    print("Enter sampling rate (1 = all points, 2 = every second point, etc.):")
    sampling_rate = int(input())
    
    # Create ground points and lines
    for i in range(0, len(points), sampling_rate):
        point = points[i]
        ground_point = np.array([point[0], point[1], 0])
        
        # Add points to the list
        lines.append([ground_point, point])
        
        # Add color based on height (normalize Z value)
        height_normalized = (point[2] - np.min(points[:, 2])) / (np.max(points[:, 2]) - np.min(points[:, 2]))
        line_colors.append([height_normalized, 0.7, 1 - height_normalized])
    
    # Create line set geometry
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.vstack([p for line in lines for p in line]))
    line_set.lines = o3d.utility.Vector2iVector([[2*i, 2*i+1] for i in range(len(lines))])
    line_set.colors = o3d.utility.Vector3dVector(np.repeat(line_colors, 1, axis=0))
    
    # Visualize
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(line_set)
    vis.add_geometry(pcd)
    
    # Set rendering options
    opt = vis.get_render_option()
    opt.point_size = 1.0
    opt.line_width = 5.0  # Increased line width from default to 5.0
    opt.background_color = np.asarray([0.1, 0.1, 0.1])
    
    vis.run()
    vis.destroy_window()
    
else:
    print("Invalid input. Please enter 1 or 2.")