import bpy
import numpy as np
import math

bpy.ops.wm.open_mainfile(filepath="boston.blend")

m = bpy.data.meshes["Grid Mesh"]
if m is None:
    raise ValueError("No mesh named 'Grid Mesh' found in the scene.")

min_x, max_x, min_y, max_y = math.inf, -math.inf, +math.inf, -math.inf
columns: dict[float, dict[float, float]] = dict()
for vertex in m.vertices:

    t = [round(c, 0) for c in vertex.co]
    min_x = min(t[0], min_x)
    max_x = max(t[0], max_x)
    min_y = min(t[1], min_y)
    max_y = max(t[1], max_y)
    
    if not t[0] in columns:
        columns[t[0]] = dict()
    columns[t[0]][t[1]] = t[2]

x_range = (min_x, max_x)
y_range = (min_y, max_y)
print("X range", x_range)
print("Y range", y_range)

# OpenTopography key:
# 80dfe841eea79f06217b2fe92a4ba8a4