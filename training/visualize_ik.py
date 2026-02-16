import numpy as np
import matplotlib.pyplot as plt
from kinematics import StretchIK
from scipy.spatial.transform import Rotation

def plot_stretch_workspace(ax, solver):
    """Draws the physical reach limits of the Stretch 3."""
    # 1. Base/Mast Center
    ax.scatter([-0.1], [0], [0], color='black', s=100, label='Mast Center')
    
    # 2. Draw the 'Sweep' of the arm (approximate reach cylinder)
    z_range = np.linspace(0.0, 1.1, 10)
    theta = np.linspace(0, 2*np.pi, 50)
    # The arm is offset approx 0.15m from mast center
    radius = 0.52 + 0.15 
    
    for z in [0.0, 1.1]: # Draw top and bottom rings
        x = radius * np.cos(theta) - 0.1
        y = radius * np.sin(theta)
        ax.plot(x, y, z, 'r--', alpha=0.3)

def visualize_target(position, quaternion):
    solver = StretchIK()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    plot_stretch_workspace(ax, solver)
    
    # Solve IK
    result = solver.solve_single(position, quaternion, debug=True)
    
    # Plot Target Point
    px, py, pz = position
    ax.scatter([px], [py], [pz], color='green' if result else 'red', s=100, label='Target')
    
    # Plot Orientation Axes (RGB = XYZ)
    r = Rotation.from_quat(quaternion).as_matrix()
    scale = 0.1
    ax.quiver(px, py, pz, r[0,0], r[1,0], r[2,0], color='r', length=scale, label='Forward (X)')
    ax.quiver(px, py, pz, r[0,1], r[1,1], r[2,1], color='g', length=scale)
    ax.quiver(px, py, pz, r[0,2], r[1,2], r[2,2], color='b', length=scale)

    ax.set_xlabel('X (Back/Front)')
    ax.set_ylabel('Y (Left/Right)')
    ax.set_zlabel('Z (Height)')
    ax.set_title(f"IK Status: {'SUCCESS' if result else 'FAILED'}")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    # Try the point that was failing for you
    target_pos = [-0.1, -0.4, 0.6]
    target_quat = [0, 0, 0, 1] 
    visualize_target(target_pos, target_quat)