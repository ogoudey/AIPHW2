import bpy
import numpy as np

bpy.ops.wm.open_mainfile(filepath="terrain.blend")



m = bpy.data.meshes("Grid Mesh")
if m is None:
    raise ValueError("No mesh named 'Grid Mesh' found in the scene.")

for vertex in m:
    pass