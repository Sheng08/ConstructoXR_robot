import numpy as np
from stl import mesh

# Load an STL file
your_mesh = mesh.Mesh.from_file('meshes/demo_model.stl')

# Find the max dimensions
max_dim = np.max(your_mesh.points, axis=0) - np.min(your_mesh.points, axis=0)
length, width, height = max_dim[0::3], max_dim[1::3], max_dim[2::3]

print(f"Length: {length} units")
print(f"Width: {width} units")
print(f"Height: {height} units")