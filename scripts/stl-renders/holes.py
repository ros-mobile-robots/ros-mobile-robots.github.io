"""Print the centres (mm, base_link frame) of small round holes in one part, by slicing it at given heights.

Usage: python holes.py <insiders path> <scene json> <link> <z1,z2,...> [max radius mm]
Used to place the screws and guide lines in jobs.py.
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

repo, scene, link = Path(sys.argv[1]), json.loads(Path(sys.argv[2]).read_text()), sys.argv[3]
heights = [float(z) for z in sys.argv[4].split(",")]
rmax = float(sys.argv[5]) if len(sys.argv) > 5 else 2.5
part = next(p for p in scene["parts"] if p["link"] == link)
mesh = trimesh.load(repo / part["mesh"], force="mesh")
mesh.apply_transform(np.array(part["matrix"]) @ np.diag(part["scale"] + [1]))
mesh.apply_scale(1000)
for z in heights:
    section = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if section is None:
        print(f"z={z}: no section")
        continue
    outline, _ = section.to_2D(to_2D=np.eye(4), check=False)
    holes = []
    for polygon in outline.polygons_full:
        for ring in polygon.interiors:
            xy = np.array(ring.coords)
            centre = xy.mean(0)
            r = np.linalg.norm(xy - centre, axis=1)
            if r.mean() < rmax and r.std() < 0.25 * r.mean():
                holes.append((round(float(centre[0]), 1), round(float(centre[1]), 1), round(float(r.mean()), 2)))
    print(f"z={z}: {sorted(holes)}")
