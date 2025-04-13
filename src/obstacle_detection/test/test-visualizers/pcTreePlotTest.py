import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData
import os

def load_ply(filepath):
    plydata = PlyData.read(filepath)
    vertices = plydata['vertex'].data
    return vertices['x'], vertices['y'], vertices['z']

def plot_quadrants():
    # Find all quadrant PLY files in current directory
    quadrant_files = [f for f in os.listdir('.') if f.startswith('quadrant_') and f.endswith('.ply')]
    
    # Setup subplot grid
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))
    axs = axs.ravel()

    # Plot each quadrant
    for i, filepath in enumerate(quadrant_files):
        x, y, z = load_ply(filepath)
        
        # 2D scatter plot for each quadrant
        axs[i].scatter(x, y, c=z, cmap='viridis', s=1)
        axs[i].set_title(f'Quadrant {i} 2D Projection')
        axs[i].set_xlabel('X')
        axs[i].set_ylabel('Y')
        plt.colorbar(axs[i].collections[0], ax=axs[i], label='Height')

    # Combined plot
    axs[4].scatter(
        np.concatenate([load_ply(f)[0] for f in quadrant_files]),
        np.concatenate([load_ply(f)[1] for f in quadrant_files]),
        c=np.concatenate([load_ply(f)[2] for f in quadrant_files]),
        cmap='viridis', 
        s=1
    )
    axs[4].set_title('Combined Quadrants 2D Projection')
    axs[4].set_xlabel('X')
    axs[4].set_ylabel('Y')
    plt.colorbar(axs[4].collections[0], ax=axs[4], label='Height')

    plt.tight_layout()
    plt.show()

plot_quadrants()