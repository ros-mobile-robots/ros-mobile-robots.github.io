"""Build the data of the URDF explorer on docs/robot-description.md.

The explorer shows DiffBot's and Remo's robot models in the browser: each link's visual shape,
collision shape, mass and frame. This script collects what the page needs, from pinned commits
of the robots' repositories:

- docs/resources/urdf-explorer/<robot>.urdf: the robot's xacro file, expanded;
- docs/resources/urdf-explorer/meshes/: the glTF files of the parts that have a model in
  robot_component_models, with the notice files that accompany them there;
- docs/resources/urdf-explorer/manifest.json: where each file comes from, with checksums;
- includes/urdf-explorer.html: the figure's markup, its data, and the tables of all links,
  which are also what the page shows without JavaScript.

The URDF stays the source for links, joints, frames, collision shapes and inertias. Only the
visual meshes are exchanged: VISUALS maps each mesh that a URDF names to the part's glTF file,
which has the same frame. A mesh without an entry is drawn as the link's collision shape.

Run from the repository root, with clones of the three repositories that contain the pinned
commits (pip install -r scripts/urdf-explorer/requirements.txt):

    python3 scripts/urdf-explorer/build.py --diffbot ~/dev/diffbot \
        --remo ~/dev/remo_description --models ~/dev/robot_component_models

The picture that the page shows before the 3D view opens, poster-diffbot.webp in the data
folder, is a screenshot of the explorer's first step and isn't written by this script.

tests/test_urdf_explorer.py checks the generated files against this script's tables.
"""

import argparse
import gzip
import hashlib
import html
import io
import json
from pathlib import Path
import posixpath
import re
import subprocess
import tarfile
import tempfile
from typing import Dict, List, NamedTuple, Optional, Set, Tuple
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent.parent
DATA_DIR = ROOT / 'docs' / 'resources' / 'urdf-explorer'
INCLUDE = ROOT / 'includes' / 'urdf-explorer.html'
DATA_URL = '/resources/urdf-explorer'
GITHUB = 'https://github.com/ros-mobile-robots'
# The code of the 3D view: its script and the files that the script imports, in VIEWER_DIR. The
# browser fetches them when the reader opens the view, together with the robot's model files.
# The size next to the button counts both. A browser test checks that the view fetches exactly
# these files (tests/browser/test_urdf_explorer_page.py)
VIEWER_DIR = ROOT / 'docs' / 'custom_theme' / 'js'
VIEWER_FILES = (
    'interactive/urdf-explorer-view.js',
    'three-0.170.0/build/three.module.min.js',
    'three-0.170.0/examples/jsm/controls/OrbitControls.js',
    'three-0.170.0/examples/jsm/loaders/GLTFLoader.js',
    'three-0.170.0/examples/jsm/utils/BufferGeometryUtils.js',
    'urdf-loader-0.13.1/URDFLoader.js',
    'urdf-loader-0.13.1/URDFClasses.js',
    'three-0.170.0/examples/jsm/loaders/STLLoader.js',
    'three-0.170.0/examples/jsm/loaders/ColladaLoader.js',
    'three-0.170.0/examples/jsm/loaders/TGALoader.js',
)


class Robot(NamedTuple):
    """Where a robot's model comes from, and the joints of its two drive wheels.

    package_dir is the folder of the description package in the repository, with a slash at
    its end, or empty if the repository is the package.
    """

    name: str
    repository: str
    commit: str
    package_dir: str
    xacro: str
    wheel_joints: Tuple[str, str]


ROBOTS = {
    'diffbot': Robot(
        name='DiffBot',
        repository='diffbot',
        commit='f5f282eed7b6a2b146d40c7d5fd641b8a339a77d',
        package_dir='diffbot_description/',
        xacro='urdf/diffbot.urdf.xacro',
        wheel_joints=('front_left_wheel_joint', 'front_right_wheel_joint'),
    ),
    'remo': Robot(
        name='Remo',
        repository='remo_description',
        commit='0f25d6d2e33ad587e476adfa8139b8cabef38413',  # Version 2.5.0
        package_dir='',
        xacro='urdf/remo.urdf.xacro',
        wheel_joints=('front_left_wheel_joint', 'front_right_wheel_joint'),
    ),
}

# robot_component_models: the commit that each part's glTF file is taken from, and the commit
# of the repository's notice file
MODELS_REPOSITORY = 'robot_component_models'
MODELS = {
    'tt_motor_wheel_65mm/tt_motor_wheel_65mm.glb': 'e2f092f',
    'swivel_caster_30mm/swivel_caster_30mm_base.glb': '4f3af14',
    'swivel_caster_30mm/swivel_caster_30mm_wheel_support.glb': '4f3af14',
    'swivel_caster_30mm/swivel_caster_30mm_wheel.glb': '4f3af14',
    'rplidar_a2/rplidar_a2.glb': '4f3af14',
    'raspberry_pi_4b/raspberry_pi_4b.glb': '4f3af14',
    'breadboard_400_point/breadboard_400_point.glb': '4f3af14',
    'teensy_4_0/teensy_4_0.glb': '4f3af14',
    'raspberry_pi_camera_v2/raspberry_pi_camera_v2.glb': '4f3af14',
}
MODELS_NOTICE_COMMIT = '4f3af14'

# The visual meshes that the URDFs name, and the part's glTF file that the explorer draws in
# their place. The glTF files are in metres and in the frame of the mesh they stand for, so the
# explorer applies the visual's origin from the URDF and no scale
DIFFBOT_MESHES = 'package://diffbot_description/meshes'
REMO_MESHES = 'package://remo_description/meshes'
CASTER = 'swivel_caster_30mm/swivel_caster_30mm'
VISUALS = {
    f'{DIFFBOT_MESHES}/components/wheel.stl': 'tt_motor_wheel_65mm/tt_motor_wheel_65mm.glb',
    f'{DIFFBOT_MESHES}/diffbot/caster_base.stl': f'{CASTER}_base.glb',
    f'{DIFFBOT_MESHES}/diffbot/caster_wheel_support.stl': f'{CASTER}_wheel_support.glb',
    f'{DIFFBOT_MESHES}/diffbot/caster_wheel.stl': f'{CASTER}_wheel.glb',
    f'{DIFFBOT_MESHES}/sensors/rplidar-a2.stl': 'rplidar_a2/rplidar_a2.glb',
    f'{REMO_MESHES}/components/wheel.stl': 'tt_motor_wheel_65mm/tt_motor_wheel_65mm.glb',
    f'{REMO_MESHES}/sensors/rplidar-a2.stl': 'rplidar_a2/rplidar_a2.glb',
    f'{REMO_MESHES}/components/raspberry_pi_4B.stl': 'raspberry_pi_4b/raspberry_pi_4b.glb',
    f'{REMO_MESHES}/components/breadboard_400_point.stl': (
        'breadboard_400_point/breadboard_400_point.glb'
    ),
    f'{REMO_MESHES}/components/teensy_4-0.stl': 'teensy_4_0/teensy_4_0.glb',
    f'{REMO_MESHES}/sensors/raspi_cam_v2_IMX219.stl': (
        'raspberry_pi_camera_v2/raspberry_pi_camera_v2.glb'
    ),
}
# The parts' names for the tables, by glTF file
PART_NAMES = {
    'tt_motor_wheel_65mm/tt_motor_wheel_65mm.glb': '65 mm wheel',
    f'{CASTER}_base.glb': 'swivel caster, base',
    f'{CASTER}_wheel_support.glb': 'swivel caster, fork',
    f'{CASTER}_wheel.glb': 'swivel caster, wheel',
    'rplidar_a2/rplidar_a2.glb': 'laser scanner',
    'raspberry_pi_4b/raspberry_pi_4b.glb': 'Raspberry Pi 4 B',
    'breadboard_400_point/breadboard_400_point.glb': 'breadboard',
    'teensy_4_0/teensy_4_0.glb': 'Teensy 4.0',
    'raspberry_pi_camera_v2/raspberry_pi_camera_v2.glb': 'camera module',
}

# The explorer's four steps: the step's id, its button and its explanation (HTML)
STEPS = [
    (
        'visual',
        'What you see',
        'The <b>visual</b> shapes are what RViz and Gazebo draw. The physics engine never looks '
        'at them. Parts with a model in <code>robot_component_models</code> show that model. '
        'The other parts are drawn as simple shapes.',
    ),
    (
        'collision',
        'What collides',
        'The <b>collision</b> shapes are what the physics engine checks for contact. They are '
        'simple on purpose: boxes, cylinders and, for the drive wheels, spheres. A sphere '
        'touches flat ground at one point, in the middle of the wheel.',
    ),
    (
        'inertia',
        'Where the mass is',
        'Each physical link has a mass, a center of mass and an inertia. Links that are only '
        'frames, such as <code>base_footprint</code>, have none. The dot is the center of mass. '
        'The box is a uniform box with the same mass and the same inertia as the link, which is '
        "how Gazebo draws inertia. It isn't the part's outline.",
    ),
    (
        'frames',
        'Frames',
        'Every link has a frame of its own: red x, green y, blue z. <code>base_footprint</code> '
        "lies on the ground under the midpoint of the drive wheels' axle, and "
        '<code>base_link</code> straight above it. For both, x points forward, y to the left '
        "and z up. Turn the wheels: a wheel's frame turns with its wheel.",
    ),
]
# What the tables and the explorer say for a visual mesh without a model. The explorer's script
# (urdf-explorer-view.js) has the same text
STAND_IN = 'simple shape (the collision shape)'
# The layers that the reader can switch: the layer's id and its label
LAYERS = [
    ('visual', 'Visual shapes'),
    ('collision', 'Collision shapes'),
    ('inertia', 'Inertia boxes'),
    ('com', 'Centers of mass'),
    ('frames', 'Frames'),
]


def git_bytes(repository: Path, spec: str) -> bytes:
    """Return a file's content at a commit (spec is commit:path)."""
    command = ['git', '-C', str(repository), 'show', spec]
    return subprocess.run(command, check=True, capture_output=True).stdout


def expand_xacro(repository: Path, robot: Robot) -> str:
    """Return the robot's URDF: its xacro file at the pinned commit, expanded."""
    import xacro  # Imported here, so the tests can use this module's tables without xacro

    prefix = robot.package_dir
    command = ['git', '-C', str(repository), 'archive', robot.commit]
    command += [f'{prefix}urdf', f'{prefix}config']
    archive = subprocess.run(command, check=True, capture_output=True).stdout
    with tempfile.TemporaryDirectory() as folder:
        tarfile.open(fileobj=io.BytesIO(archive)).extractall(folder, filter='data')
        package = Path(folder) / prefix
        # The xacro files find their package with $(find ...), which needs a ROS installation.
        # The package is this folder
        for path in (package / 'urdf').rglob('*.xacro'):
            text = path.read_text(encoding='utf-8')
            text = text.replace('$(find ${package_name})', str(package))
            path.write_text(text, encoding='utf-8')
        document = xacro.process_file(str(package / robot.xacro))
    # Without the XML comments: the explorer reads elements only, and xacro's own header comment
    # names the temporary folder, which changes from run to run
    urdf = re.sub(r'<!--.*?-->', '', document.toprettyxml(indent='  '), flags=re.DOTALL)
    return '\n'.join(line for line in urdf.splitlines() if line.strip()) + '\n'


def mesh_uri(filename: str) -> str:
    """Return a mesh's package:// URI without . and .. segments."""
    scheme, rest = filename.split('://', 1)
    return f'{scheme}://{posixpath.normpath(rest)}'


def child(element: ET.Element, tag: str) -> ET.Element:
    """Return an element's child with this tag, which the URDF must have."""
    found = element.find(tag)
    if found is None:
        raise ValueError(f'<{element.tag}> without <{tag}>')
    return found


def attribute(element: ET.Element, name: str) -> str:
    """Return an element's attribute, which the URDF must have."""
    value = element.get(name)
    if value is None:
        raise ValueError(f'<{element.tag}> without {name}')
    return value


def millimetres(metres: float) -> str:
    """Return a length in mm, without trailing zeros."""
    return f'{metres * 1000:g}'


def shape_text(element: ET.Element) -> str:
    """Describe the geometry of a visual or collision element in words, with sizes in mm."""
    shape = child(element, 'geometry')[0]
    if shape.tag == 'box':
        sides = [millimetres(float(side)) for side in attribute(shape, 'size').split()]
        return 'box ' + ' × '.join(sides) + ' mm'
    if shape.tag == 'sphere':
        radius = millimetres(float(attribute(shape, 'radius')))
        return f'sphere, radius {radius} mm'
    if shape.tag == 'cylinder':
        radius = millimetres(float(attribute(shape, 'radius')))
        length = millimetres(float(attribute(shape, 'length')))
        return f'cylinder, radius {radius} mm, length {length} mm'
    return 'mesh'


def visual_text(visual: Optional[ET.Element]) -> str:
    """Say what the explorer draws for a link's visual: a part's model or a shape.

    A mesh without a model is drawn as the link's collision shape, which STAND_IN says.
    """
    if visual is None:
        return 'none'
    shape = child(visual, 'geometry')[0]
    if shape.tag != 'mesh':
        return shape_text(visual)
    model = VISUALS.get(mesh_uri(attribute(shape, 'filename')))
    return f'model: {PART_NAMES[model]}' if model else STAND_IN


def link_rows(urdf: str) -> List[Dict[str, str]]:
    """Return one table row per link: its parent, joint, mass, visual and collision."""
    robot = ET.fromstring(urdf)
    joints = {attribute(child(joint, 'child'), 'link'): joint for joint in robot.findall('joint')}
    rows = []
    for link in robot.findall('link'):
        name = attribute(link, 'name')
        joint = joints.get(name)
        parent = 'none: the root'
        if joint is not None:
            parent = attribute(child(joint, 'parent'), 'link')
        collision = link.find('collision')
        inertial = link.find('inertial')
        mass = 'none'
        if inertial is not None:
            grams = float(attribute(child(inertial, 'mass'), 'value')) * 1000
            mass = f'{grams:g} g'
        rows.append({
            'link': name,
            'parent': parent,
            'joint': '' if joint is None else attribute(joint, 'type'),
            'mass': mass,
            'visual': visual_text(link.find('visual')),
            'collision': 'none' if collision is None else shape_text(collision),
        })
    return rows


def check_inertias(name: str, urdf: str) -> None:
    """Stop if a link's inertia is something the explorer doesn't draw.

    The explorer draws a uniform box with the link's mass and its three principal moments. That
    needs a positive mass, no products of inertia, and moments that a real body can have: each
    one at most the sum of the other two.
    """
    for link in ET.fromstring(urdf).findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        values = {key: float(value) for key, value in child(inertial, 'inertia').attrib.items()}
        mass = float(attribute(child(inertial, 'mass'), 'value'))
        moments = sorted((values['ixx'], values['iyy'], values['izz']))
        where = name + ', ' + attribute(link, 'name')
        assert mass > 0 and moments[0] > 0, where
        assert all(abs(values.get(key, 0.0)) < 1e-12 for key in ('ixy', 'ixz', 'iyz')), where
        assert moments[2] <= (moments[0] + moments[1]) * (1 + 1e-9), where


def transfer_bytes(data: bytes) -> int:
    """Return about how many bytes the site sends for a file.

    The site's host, GitHub Pages, sends the scripts, the URDF files and the glTF files
    compressed with gzip. Its sizes are within 2 % of this function's for the explorer's files
    (compared with curl on a pull request's preview).
    """
    return len(gzip.compress(data, compresslevel=6, mtime=0))


def viewer_bytes() -> int:
    """Return about how many bytes the site sends for the 3D view's code."""
    return sum(transfer_bytes((VIEWER_DIR / file).read_bytes()) for file in VIEWER_FILES)


def megabytes(size: int) -> str:
    """Return a size in MB with one decimal."""
    return f'{size / 1e6:.1f} MB'


def sha256(data: bytes) -> str:
    """Return the SHA-256 checksum of some bytes."""
    return hashlib.sha256(data).hexdigest()


def table(name: str, rows: List[Dict[str, str]]) -> str:
    """Return the HTML table of a robot's links."""
    columns = ('Link', 'Parent link', 'Joint', 'Mass', 'Visual', 'Collision shape')
    head = ''.join(f'<th scope="col">{column}</th>' for column in columns)
    body = ''
    for row in rows:
        keys = ('parent', 'joint', 'mass', 'visual', 'collision')
        cells = [html.escape(row[key]) for key in keys]
        body += f'<tr><th scope="row"><code>{html.escape(row["link"])}</code></th>'
        body += ''.join(f'<td>{cell}</td>' for cell in cells) + '</tr>'
    return (
        f'<table class="ix-table"><caption>{name}: {len(rows)} links</caption>'
        f'<thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>'
    )


def view_buttons() -> str:
    """Return the camera buttons: four views, turning, zooming and enlarging."""
    views = (('3d', '3D'), ('front', 'Front'), ('side', 'Side'), ('top', 'Top'))
    buttons = ''.join(
        f'<button type="button" data-view="{view}" aria-pressed="{str(view == "3d").lower()}">'
        f'{label}</button>'
        for view, label in views
    )
    return (
        f'<div class="ix-viewbar" role="group" aria-label="Camera">{buttons}'
        '<button type="button" data-turn="-1" aria-label="Turn the view to the left">⟲</button>'
        '<button type="button" data-turn="1" aria-label="Turn the view to the right">⟳</button>'
        '<button type="button" data-zoom="1" aria-label="Zoom in">+</button>'
        '<button type="button" data-zoom="-1" aria-label="Zoom out">−</button>'
        '<button type="button" class="ix-enlarge">Enlarge</button></div>'
    )


def include(
    visuals: Dict[str, Dict[str, Optional[str]]],
    rows: Dict[str, List[Dict[str, str]]],
    sizes: Dict[str, int],
) -> str:
    """Return the include's HTML: the figure, its data and the tables of all links.

    visuals has, per robot, the glTF file name of each visual mesh URI, or None for a mesh
    that is drawn as a shape. sizes has about the bytes that the site sends when a robot's 3D
    view opens: the view's code and the robot's model files, which are its URDF file and its
    parts' glTF files, all compressed (transfer_bytes).
    """
    data = {
        'robots': {
            key: {
                'name': robot.name,
                'urdf': f'{key}.urdf',
                'wheelJoints': list(robot.wheel_joints),
                'visuals': visuals[key],
                'source': f'{robot.repository} at {robot.commit[:7]}',
            }
            for key, robot in ROBOTS.items()
        },
        'parts': {posixpath.basename(file): name for file, name in PART_NAMES.items()},
        'steps': [{'id': step, 'title': title} for step, title, _ in STEPS],
    }
    shown_sizes = {key: megabytes(size) for key, size in sizes.items()}
    options = ''.join(
        f'<option value="{key}">{robot.name}</option>' for key, robot in ROBOTS.items()
    )
    steps = ''.join(
        f'<button type="button" data-step="{step}" aria-pressed="{str(index == 0).lower()}">'
        f'{index + 1}. {title}</button>'
        for index, (step, title, _) in enumerate(STEPS)
    )
    texts = ''.join(
        f'<p data-step-text="{step}"{"" if index == 0 else " hidden"}>{text}</p>'
        for index, (step, _, text) in enumerate(STEPS)
    )
    layers = ''.join(
        f'<label><input type="checkbox" data-layer="{layer}"> {label}</label>'
        for layer, label in LAYERS
    )
    sources = ', '.join(
        f'<a href="{GITHUB}/{robot.repository}/tree/{robot.commit}">{robot.name} at '
        f'<code>{robot.commit[:7]}</code></a>'
        for robot in ROBOTS.values()
    )
    lines = [
        '<figure class="ix ix-urdf" data-ix="urdf-explorer">',
        '<script type="application/json" class="ix-data">'
        f'{json.dumps(data, separators=(",", ":"))}</script>',
        '<div class="ix-stage">',
        '<div class="ix-view3d">',
        f'<img class="ix-poster off-glb" src="{DATA_URL}/poster-diffbot.webp" width="1200" '
        'height="800" alt="DiffBot\'s model from the front left: two yellow drive wheels, a '
        'small caster wheel at the back and the laser scanner on top" loading="lazy">',
        '<div class="ix-view3d-start" data-live hidden>'
        f'<label>Robot <select class="ix-robot-select">{options}</select></label>'
        '<button type="button" class="ix-open">Open the 3D view</button>'
        f'<small class="ix-download" data-sizes="{html.escape(json.dumps(shown_sizes))}" '
        'title="The code of the 3D view and the robot\'s model files, compressed as this site '
        'sends them"></small>'
        '</div>',
        '<p class="ix-status" role="status" hidden></p>',
        '</div>',
        '<div class="ix-explorer" hidden>',
        view_buttons(),
        f'<div class="ix-steps" role="group" aria-label="Four steps">{steps}</div>',
        f'<div class="ix-step-texts">{texts}</div>',
        '<div class="ix-panels">',
        '<div class="ix-panel"><p class="ix-group">Links</p><div class="ix-tree"></div></div>',
        '<div class="ix-panel"><p class="ix-group">Selected link</p>'
        '<div class="ix-info" aria-live="polite"></div></div>',
        '</div>',
        '<details class="ix-settings"><summary>Choose the layers yourself</summary>'
        '<div class="ix-controls">',
        f'<div class="ix-layers">{layers}</div>',
        '<div class="ix-control"><label for="urdf-explorer-wheels">Turn the drive wheels</label>'
        '<output for="urdf-explorer-wheels">0°</output><input type="range" '
        'id="urdf-explorer-wheels" class="ix-wheels" min="-180" max="180" step="5" value="0" '
        'aria-describedby="urdf-explorer-wheels-help"><small id="urdf-explorer-wheels-help">'
        "Turning a joint moves the link and its frame. The robot doesn't drive: nothing is "
        'simulated here.</small></div>',
        '<div><button type="button" class="ix-reset">Reset</button></div>',
        '</div></details>',
        '</div>',
        '</div>',
        '<figcaption>The robot models as their URDF files describe them.<span '
        'class="ix-view-hint" hidden> Orange: collision shapes. One finger or the mouse turns '
        'the view. Two fingers or Ctrl and the mouse wheel zoom.</span></figcaption>',
        '<details class="ix-link-tables"><summary>All links as tables</summary>',
        *(table(ROBOTS[key].name, rows[key]) for key in ROBOTS),
        '</details>',
        '<p class="ix-model-note"><b>What this figure shows.</b> The values of the URDF files, '
        f'not measurements: {sources}. Nothing moves by itself and no physics is computed. The '
        f'parts\' models are those of <a href="{GITHUB}/{MODELS_REPOSITORY}">'
        '<code>robot_component_models</code></a>.</p>',
        '</figure>',
    ]
    return '\n'.join(lines) + '\n'


def main() -> None:
    """Write the URDF files, the glTF files, the manifest and the include."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--diffbot', type=Path, required=True, help='a clone of diffbot')
    parser.add_argument('--remo', type=Path, required=True, help='a clone of remo_description')
    parser.add_argument(
        '--models', type=Path, required=True, help='a clone of robot_component_models'
    )
    arguments = parser.parse_args()
    clones = {'diffbot': arguments.diffbot, 'remo_description': arguments.remo}

    (DATA_DIR / 'meshes').mkdir(parents=True, exist_ok=True)
    manifest_robots = {}
    manifest_meshes = {}
    visuals: Dict[str, Dict[str, Optional[str]]] = {}
    rows = {}
    model_bytes = {}
    used: Set[str] = set()
    for key, robot in ROBOTS.items():
        urdf = expand_xacro(clones[robot.repository], robot)
        check_inertias(key, urdf)
        (DATA_DIR / f'{key}.urdf').write_text(urdf, encoding='utf-8')
        uris = {mesh_uri(attribute(mesh, 'filename')) for mesh in ET.fromstring(urdf).iter('mesh')}
        models = {uri: VISUALS.get(uri) for uri in sorted(uris)}
        used.update(file for file in models.values() if file)
        visuals[key] = {
            uri: posixpath.basename(file) if file else None for uri, file in models.items()
        }
        rows[key] = link_rows(urdf)
        manifest_robots[key] = {
            'urdf': f'{key}.urdf',
            'sha256': sha256(urdf.encode('utf-8')),
            'source': {
                'repository': f'{GITHUB}/{robot.repository}',
                'commit': robot.commit,
                'file': robot.package_dir + robot.xacro,
            },
            'visuals': models,
        }
        model_bytes[key] = transfer_bytes(urdf.encode('utf-8'))
    assert used == set(MODELS), sorted(used ^ set(MODELS))
    assert set(VISUALS.values()) == set(MODELS)

    mesh_bytes = {}
    for file, commit in sorted(MODELS.items()):
        path = f'meshes/{file}'
        data = git_bytes(arguments.models, f'{commit}:{path}')
        name = posixpath.basename(file)
        (DATA_DIR / 'meshes' / name).write_bytes(data)
        notice = git_bytes(arguments.models, f'{commit}:{path}.license')
        (DATA_DIR / 'meshes' / f'{name}.license').write_bytes(notice)
        command = ['git', '-C', str(arguments.models), 'rev-parse', commit]
        full_commit = subprocess.run(command, check=True, capture_output=True, text=True).stdout
        mesh_bytes[name] = transfer_bytes(data)
        manifest_meshes[name] = {
            'source': {
                'repository': f'{GITHUB}/{MODELS_REPOSITORY}',
                'commit': full_commit.strip(),
                'file': path,
            },
            'bytes': len(data),
            'sha256': sha256(data),
        }
    notice = git_bytes(arguments.models, f'{MODELS_NOTICE_COMMIT}:LICENSE')
    (DATA_DIR / 'meshes' / 'LICENSE').write_bytes(notice)
    for key in ROBOTS:
        model_bytes[key] += sum(mesh_bytes[file] for file in set(visuals[key].values()) if file)
    sizes = {key: size + viewer_bytes() for key, size in model_bytes.items()}

    manifest = {'robots': manifest_robots, 'meshes': manifest_meshes}
    text = json.dumps(manifest, indent=1) + '\n'
    (DATA_DIR / 'manifest.json').write_text(text, encoding='utf-8')
    output = include(visuals, rows, sizes)
    # MkDocs inserts the include into a Markdown page: a blank line would end its HTML block, and
    # these character pairs would start a template expression
    assert '\n\n' not in output and not any(pair in output for pair in ('{{', '{#', '{%'))
    INCLUDE.write_text(output, encoding='utf-8')
    for key, robot in ROBOTS.items():
        with_model = sum(1 for file in visuals[key].values() if file)
        print(
            f'{robot.name}: {len(rows[key])} links, {with_model} of {len(visuals[key])} visual '
            f'meshes have a model, the view fetches about {megabytes(sizes[key])} compressed '
            f'({model_bytes[key] / 1e6:.2f} MB of model files and {viewer_bytes() / 1e6:.2f} MB '
            'of code)'
        )
    print(f'wrote {DATA_DIR.relative_to(ROOT)} and {INCLUDE.relative_to(ROOT)}')


if __name__ == '__main__':
    main()
