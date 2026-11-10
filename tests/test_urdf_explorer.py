"""Tests for scripts/urdf-explorer/build.py and the files that it wrote for the URDF explorer.

Run from the repository root: python -m unittest discover -s tests
"""
import hashlib
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'urdf-explorer'))

import build  # noqa: E402

MANIFEST = json.loads((build.DATA_DIR / 'manifest.json').read_text(encoding='utf-8'))


def include_data():
    """Return the data that the include gives the explorer's script."""
    text = build.INCLUDE.read_text(encoding='utf-8')
    start = text.index('class="ix-data">') + len('class="ix-data">')
    return json.loads(text[start:text.index('</script>', start)])


class TestRobotFiles(unittest.TestCase):
    """The URDF files are the ones that the manifest describes."""

    def test_urdf_files_match_the_manifest(self):
        for key, robot in MANIFEST['robots'].items():
            data = (build.DATA_DIR / robot['urdf']).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), robot['sha256'], key)
            self.assertEqual(robot['source']['commit'], build.ROBOTS[key].commit, key)

    def test_link_counts(self):
        counts = {key: len(ET.parse(build.DATA_DIR / robot['urdf']).getroot().findall('link'))
                  for key, robot in MANIFEST['robots'].items()}
        self.assertEqual(counts, {'diffbot': 11, 'remo': 25})

    def test_base_footprint_lies_under_the_axle_midpoint(self):
        # The explorer's fourth step says so: the two drive wheels' joints are equally far left
        # and right of the chassis frame, and the chassis joint moves them back onto base_link
        for key, robot in MANIFEST['robots'].items():
            root = ET.parse(build.DATA_DIR / robot['urdf']).getroot()
            origins = {}
            for joint in root.findall('joint'):
                xyz = joint.find('origin').get('xyz').split()
                origins[joint.get('name')] = [float(value) for value in xyz]
            left, right = (origins[name] for name in build.ROBOTS[key].wheel_joints)
            chassis = origins['chassis_joint']
            self.assertAlmostEqual(chassis[0] + (left[0] + right[0]) / 2, 0.0, places=9, msg=key)
            self.assertAlmostEqual(chassis[1] + (left[1] + right[1]) / 2, 0.0, places=9, msg=key)

    def test_inertias_can_be_drawn_as_boxes(self):
        for key, robot in MANIFEST['robots'].items():
            build.check_inertias(key, (build.DATA_DIR / robot['urdf']).read_text(encoding='utf-8'))


class TestMeshes(unittest.TestCase):
    """The mesh folder holds exactly the parts' glTF files of the script's table."""

    def test_only_the_listed_files_are_in_the_mesh_folder(self):
        names = {Path(file).name for file in build.MODELS}
        expected = names | {f'{name}.license' for name in names} | {'LICENSE'}
        self.assertEqual({path.name for path in (build.DATA_DIR / 'meshes').iterdir()}, expected)

    def test_mesh_files_match_the_manifest(self):
        self.assertEqual(set(MANIFEST['meshes']), {Path(file).name for file in build.MODELS})
        for name, mesh in MANIFEST['meshes'].items():
            data = (build.DATA_DIR / 'meshes' / name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), mesh['sha256'], name)
            self.assertEqual(data[:4], b'glTF', name)
            self.assertTrue(mesh['source']['repository'].endswith('/robot_component_models'), name)

    def test_every_visual_mesh_of_the_urdfs_is_in_the_table_or_drawn_as_a_shape(self):
        data = include_data()
        for key, robot in MANIFEST['robots'].items():
            root = ET.parse(build.DATA_DIR / robot['urdf']).getroot()
            uris = {build.mesh_uri(mesh.get('filename')) for mesh in root.iter('mesh')}
            self.assertEqual(set(data['robots'][key]['visuals']), uris, key)
            for uri, file in data['robots'][key]['visuals'].items():
                if file is not None:
                    self.assertTrue((build.DATA_DIR / 'meshes' / file).is_file(), uri)
                    self.assertEqual(Path(build.VISUALS[uri]).name, file, uri)


class TestInclude(unittest.TestCase):
    """MkDocs can insert the include, and its tables list every link."""

    def test_include_has_no_blank_line_or_template_marker(self):
        text = build.INCLUDE.read_text(encoding='utf-8')
        self.assertNotIn('\n\n', text)
        for pair in ('{{', '{#', '{%'):
            self.assertNotIn(pair, text)

    def test_tables_list_every_link(self):
        text = build.INCLUDE.read_text(encoding='utf-8')
        for robot in MANIFEST['robots'].values():
            for link in ET.parse(build.DATA_DIR / robot['urdf']).getroot().findall('link'):
                self.assertIn(f'<code>{link.get("name")}</code>', text)


if __name__ == '__main__':
    unittest.main()
