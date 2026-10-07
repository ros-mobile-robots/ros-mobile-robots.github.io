"""Expand Remo's xacro and write every visual's mesh and pose to build/scene_<sbc>_<camera>.json.

Usage: python scene.py <path to remo_description_insiders>
Poses are in metres relative to base_link; mesh paths are relative to the insiders repo.
"""
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import xacro
import yourdfpy

repo = Path(sys.argv[1]).resolve()
build = Path(__file__).parent / "build"
pkg = build / "pkg"
shutil.rmtree(pkg, ignore_errors=True)
shutil.copytree(repo / "urdf", pkg / "urdf")
shutil.copytree(repo / "config", pkg / "config")
for f in (pkg / "urdf").rglob("*.xacro"):
    f.write_text(f.read_text().replace("$(find ${package_name})", str(pkg)))


def resolve(filename):
    return filename.replace("package://remo_description/", f"{repo}/")


for sbc in ["rpi"]:
    for camera in ["raspi-cam", "oak-1", "oak-d"]:
        doc = xacro.process_file(str(pkg / "urdf/remo.urdf.xacro"),
                                 mappings={"camera_type": f"'{camera}'", "sbc_type": f"'{sbc}'"})
        urdf = build / f"remo_{sbc}_{camera}.urdf"
        urdf.write_text(doc.toprettyxml(indent="  "))
        robot = yourdfpy.URDF.load(str(urdf), load_meshes=False, filename_handler=resolve)
        parts = []
        for link in robot.robot.links:
            link_pose = robot.get_transform(link.name, robot.base_link)
            for visual in link.visuals:
                mesh = visual.geometry.mesh if visual.geometry is not None else None
                if mesh is None:
                    continue
                pose = link_pose @ (visual.origin if visual.origin is not None else np.eye(4))
                scale = np.broadcast_to(np.asarray(mesh.scale if mesh.scale is not None else 1.0, float), (3,))
                parts.append({"link": link.name, "mesh": str(Path(resolve(mesh.filename)).relative_to(repo)),
                              "matrix": pose.tolist(), "scale": scale.tolist()})
        (build / f"scene_{sbc}_{camera}.json").write_text(json.dumps({"parts": parts}, indent=1))
        print(f"{sbc} {camera}: {len(parts)} parts")
