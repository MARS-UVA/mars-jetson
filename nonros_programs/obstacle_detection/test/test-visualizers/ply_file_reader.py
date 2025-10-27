import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from plyfile import PlyData
import open3d as o3d
def median_filter(data, kernel_size):
    """
    Applies a median filter to the input data.
    
    Args:
    data (numpy.ndarray): Input data.
    kernel_size (int): Size of the median filter kernel.
    
    Returns:
    numpy.ndarray: Filtered data.1
    """
    filtered_data = np.zeros_like(data)
    
    for i in range(kernel_size // 2, len(data) - kernel_size // 2):
        start = i - kernel_size // 2
        end = i + kernel_size // 2 + 1
        filtered_data[i] = np.median(data[start:end])
    
    return filtered_data
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
    plydata = PlyData.read(ply_path)

    vertices = plydata['vertex'].data

    x = vertices['x']
    y = vertices['y']
    z = vertices['z']

    # Apply 4x4 median filter for downsampling
    x_downsampled = median_filter(x, 10)
    y_downsampled = median_filter(y, 10)
    z_downsampled = median_filter(z, 10)

    if viewer_in == 2:
        # Create 2D visualization with Seaborn
        plt.figure()
        
        # Create scatter plot with Seaborn
        scatter = plt.scatter(x_downsampled, y_downsampled, c=z_downsampled, cmap='viridis', s=1, alpha=0.6)
        
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
        # Create 3D visualization with Seaborn styling
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Create scatter plot
        scatter = ax.scatter(x_downsampled, y_downsampled, z_downsampled, c=z_downsampled, cmap='viridis', s=1, alpha=0.6)
        
        # Add colorbar with Seaborn styling
        cbar = plt.colorbar(scatter)
        cbar.set_label('Height', fontsize=12)
        
        # Customize plot appearance
        ax.set_title('Point Cloud Visualization', fontsize=14, pad=20)
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
    pcd = o3d.io.read_point_cloud(ply_path)
    
    # Apply 4x4 median filter for downsampling
    downsampled_pcd = pcd.uniform_down_sample(4)
    
    # Open3D visualization remains the same
    o3d.visualization.draw_geometries([downsampled_pcd])
    
    downsampled_pcd.normalize_normals()
    downsampled_pcd.paint_uniform_color([1, 0.706, 0])
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(downsampled_pcd)
    vis.run()
    
else:
    print("Invalid input. Please enter 1 or 2.")