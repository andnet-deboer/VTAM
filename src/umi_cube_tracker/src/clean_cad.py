import trimesh
import numpy as np

# Load your UMI STL
mesh = trimesh.load("umi.stl", force="mesh")

# 1. Center the mesh
vertices = np.array(mesh.vertices)
vertices -= (vertices.min(axis=0) + vertices.max(axis=0)) / 2.0

# 2. SCALE TO METERS (Crucial for ViSP)
vertices /= 1000.0 

# 3. Simplify: We only want the bounding box edges for tracking stability
# If you want the full mesh, use mesh.faces. For stability, use this box logic:
h = 0.085 / 2.0 # Half-width of your 85mm cube
box_v = np.array([
    [ h,  h,  h], [ h, -h,  h], [-h, -h,  h], [-h,  h,  h],
    [-h,  h, -h], [-h, -h, -h], [ h, -h, -h], [ h,  h, -h]
])

# 4. Save CAO
with open("umi.cao", "w") as f:
    f.write(f"V1\n# 3D Points\n{len(box_v)}\n")
    for v in box_v:
        f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
    f.write("# 3D Lines\n12\n0 1\n1 2\n2 3\n3 0\n4 5\n5 6\n6 7\n7 4\n0 7\n1 6\n2 5\n3 4\n")
    f.write("# Faces from 3D lines\n5\n") # Bottom is hollow, so only 5 faces
    f.write("4 3 2 1 0\n4 0 1 6 7\n4 1 2 5 6\n4 2 3 4 5\n4 3 0 7 4\n")
    f.write("# Faces from 3D points\n0\n# 3D cylinders\n0\n# 3D circles\n0\n")

print("Simplified Meter-scale CAO generated.")