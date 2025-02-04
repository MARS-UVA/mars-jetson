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

print("Robot path PLY file path (press Enter to skip)?")
ply_path3 = input()

has_path = len(ply_path3.strip()) > 0

if viewer_in == 1:
    print("2D or 3D representation (2 for 2D or 3 for 3D)?")
    viewer_in = int(input())
    
    plydata1 = PlyData.read(ply_path1)
    plydata2 = PlyData.read(ply_path2)
    if has_path:
        plydata3 = PlyData.read(ply_path3)

    vertices1 = plydata1['vertex'].data
    vertices2 = plydata2['vertex'].data
    if has_path:
        vertices3 = plydata3['vertex'].data

    x1, y1, z1 = vertices1['x'], vertices1['y'], vertices1['z']
    x2, y2, z2 = vertices2['x'], vertices2['y'], vertices2['z']
    if has_path:
        x3, y3, z3 = vertices3['x'], vertices3['y'], vertices3['z']

    if viewer_in == 2:
        plt.figure()
        
        # Reduced alpha to 0.2 for first point cloud
        scatter1 = plt.scatter(x1, y1, c=z1, cmap='viridis', s=1, alpha=0.2, 
                             label='Input Point Cloud')
        scatter2 = plt.scatter(x2, y2, c='red', s=10, alpha=0.8, 
                             label='Classified Obstacles')
        
        if has_path:
            # Draw parallel lines along the path
            for i in range(len(x3)-1):
                # Calculate perpendicular vector
                dx = x3[i+1] - x3[i]
                dy = y3[i+1] - y3[i]
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    # Normalize and rotate 90 degrees
                    perpx = -dy/length * 0.1
                    perpy = dx/length * 0.1
                    
                    # Draw main path
                    plt.plot([x3[i], x3[i+1]], [y3[i], y3[i+1]], 'k-', linewidth=2)
                    
                    # Draw parallel lines
                    plt.plot([x3[i] + perpx, x3[i+1] + perpx], 
                            [y3[i] + perpy, y3[i+1] + perpy], 'k--', linewidth=1)
                    plt.plot([x3[i] - perpx, x3[i+1] - perpx], 
                            [y3[i] - perpy, y3[i+1] - perpy], 'k--', linewidth=1)
            
            # Plot path points
            scatter3 = plt.scatter(x3, y3, c='black', s=10, alpha=0.8,
                                 label='Robot Path')
            
            # Add larger circle at destination
            plt.scatter(-0.5, 1.5, c='black', s=50, alpha=0.8,
                      label='Destination')
        
        cbar = plt.colorbar(scatter1)
        cbar.set_label('Height from ground (Input Point Cloud)', fontsize=10)
        
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

        # Reduced alpha to 0.2 for first point cloud
        scatter1 = ax.scatter(x1, y1, z1, c=z1, cmap='viridis', s=1, alpha=0.1,
                            label='Input Point Cloud')
        scatter2 = ax.scatter(x2, y2, z2, c=z2, cmap='viridis', s=1, alpha=0.8,
                            label='Classified Obstacles')
        
        if has_path:
            # Draw parallel lines along the path
            for i in range(len(x3)-1):
                # Calculate perpendicular vector in xy plane
                dx = x3[i+1] - x3[i]
                dy = y3[i+1] - y3[i]
                dz = z3[i+1] - z3[i]
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    # Normalize and rotate 90 degrees in xy plane
                    perpx = -dy/length * 0.1
                    perpy = dx/length * 0.1
                    
                    # Draw main path
                    ax.plot([x3[i], x3[i+1]], [y3[i], y3[i+1]], [z3[i], z3[i+1]],
                           'k-', linewidth=2)
                    
                    # Draw parallel lines
                    ax.plot([x3[i] + perpx, x3[i+1] + perpx], 
                           [y3[i] + perpy, y3[i+1] + perpy],
                           [z3[i], z3[i+1]], 'k--', linewidth=1)
                    ax.plot([x3[i] - perpx, x3[i+1] - perpx], 
                           [y3[i] - perpy, y3[i+1] - perpy],
                           [z3[i], z3[i+1]], 'k--', linewidth=1)
            
            # Plot path points
            scatter3 = ax.scatter(x3, y3, z3, c='black', s=10, alpha=0.8,
                                label='Robot Path')
            
            # Add larger circle at destination
            ax.scatter(x3[-1], y3[-1], z3[-1], c='black', s=100, alpha=0.8,
                      label='Destination')
        
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
    if has_path:
        pcd3 = o3d.io.read_point_cloud(ply_path3)
        
        # Create line set for path and parallel lines
        points = np.asarray(pcd3.points)
        lines = [[i, i+1] for i in range(len(points)-1)]
        colors = [[0, 0, 0] for _ in range(len(lines))]  # black lines
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        # Create parallel lines
        parallel_points = []
        parallel_lines = []
        for i in range(len(points)-1):
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            length = np.sqrt(dx**2 + dy**2)
            if length > 0:
                perpx = -dy/length * 0.1
                perpy = dx/length * 0.1
                
                # Points for first parallel line
                p1 = points[i] + np.array([perpx, perpy, 0])
                p2 = points[i+1] + np.array([perpx, perpy, 0])
                # Points for second parallel line
                p3 = points[i] - np.array([perpx, perpy, 0])
                p4 = points[i+1] - np.array([perpx, perpy, 0])
                
                idx = len(parallel_points)
                parallel_points.extend([p1, p2, p3, p4])
                parallel_lines.extend([[idx, idx+1], [idx+2, idx+3]])

    pcd1.normalize_normals()
    # Set lower opacity color for first point cloud (alpha=0.2)
    pcd1.paint_uniform_color([1, 0.706, 0])
    pcd1.colors = o3d.utility.Vector3dVector(np.asarray(pcd1.colors) * 0.2)
    pcd2.paint_uniform_color([1, 0, 0])
    if has_path:
        pcd3.paint_uniform_color([0, 0, 0])
        
        # Create sphere for destination point
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        sphere.translate(points[-1])
        sphere.paint_uniform_color([0, 0, 0])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    if has_path:
        vis.add_geometry(pcd3)
        vis.add_geometry(line_set)
        vis.add_geometry(sphere)
        
        # Add parallel lines
        parallel_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(parallel_points),
            lines=o3d.utility.Vector2iVector(parallel_lines))
        parallel_line_set.colors = o3d.utility.Vector3dVector(
            [[0, 0, 0] for _ in range(len(parallel_lines))])
        vis.add_geometry(parallel_line_set)
        
    vis.run()
    vis.destroy_window()
    
else:
    print("Invalid input. Please enter 1 or 2.")