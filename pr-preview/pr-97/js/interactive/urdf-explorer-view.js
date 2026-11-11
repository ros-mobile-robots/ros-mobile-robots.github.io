// The 3D view of the URDF explorer. urdf-explorer.js imports this file when the reader opens the
// view. three.js draws, urdf-loader reads the URDF (links, joints, visual and collision shapes,
// inertial values). This file adds what is specific to the lesson: the layers and the four
// steps, the inertia boxes, the frames, the list of links and the selected link's values.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import URDFLoader from 'urdf-loader';

// Camera positions in metres, in the robot's frame: x forward, y left, z up. 3D looks from the
// front left, Side from the right, so the robot's front points to the right. Top: the small
// negative x keeps the robot's front up and its left on the left of the screen
const VIEWS = { '3d': [0.26, 0.28, 0.21], front: [0.46, 0, 0.09], side: [0, -0.46, 0.09], top: [-0.0001, 0, 0.52] };
const TARGET = new THREE.Vector3(-0.02, 0, 0.065);
// The layers that each of the four steps shows, and the link that it selects
const STEPS = {
  visual: { layers: ['visual'], select: null },
  collision: { layers: ['visual', 'collision'], select: 'front_left_wheel' },
  inertia: { layers: ['visual', 'inertia', 'com'], select: 'front_left_wheel' },
  frames: { layers: ['visual', 'frames'], select: 'front_left_wheel' },
};
const COLORS = { collision: 0xe8590c, inertia: 0x7048e8, com: 0x0f8a5f, standIn: 0x9aa0a6, selected: 0x1f6fd0 };

const millimetres = (metres) => `${+(metres * 1000).toFixed(1)}`;
const reducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

/** A mesh's package:// address without . and .. parts, as in the include's table of visuals. */
function meshAddress(path) {
  const [scheme, rest] = path.split('://');
  const parts = [];
  for (const part of rest.split('/')) {
    if (part === '..') parts.pop(); else if (part !== '.' && part !== '') parts.push(part);
  }
  return `${scheme}://${parts.join('/')}`;
}

/** The text of a URDF geometry element, with sizes in mm. */
function shapeText(element) {
  const shape = element && element.querySelector('geometry > *');
  if (!shape) return 'none';
  const values = (name) => shape.getAttribute(name).trim().split(/\s+/).map(Number);
  if (shape.tagName === 'box') return `box ${values('size').map(millimetres).join(' × ')} mm`;
  if (shape.tagName === 'sphere') return `sphere, radius ${millimetres(values('radius')[0])} mm`;
  if (shape.tagName === 'cylinder') return `cylinder, radius ${millimetres(values('radius')[0])} mm, length ${millimetres(values('length')[0])} mm`;
  return 'mesh';
}

/**
 * The size of the uniform box that has a mass and three principal moments of inertia, or null
 * if the values aren't those of a real body or the inertia has products of inertia, which this
 * view doesn't draw. A box with the sides x, y, z has Ixx = m (y^2 + z^2) / 12.
 */
export function inertiaBox(inertial) {
  const { mass, inertia } = inertial;
  const { ixx, iyy, izz } = inertia;
  const products = [inertia.ixy, inertia.ixz, inertia.iyz].some((value) => Math.abs(value) > 1e-12);
  const sorted = [ixx, iyy, izz].sort((a, b) => a - b);
  if (!(mass > 0) || !(sorted[0] > 0) || products || sorted[2] > (sorted[0] + sorted[1]) * (1 + 1e-9)) return null;
  const side = (a, b, c) => Math.sqrt(Math.max(6 * (a + b - c) / mass, 0));
  return [side(iyy, izz, ixx), side(ixx, izz, iyy), side(ixx, iyy, izz)];
}

export async function openExplorer(root, data, dataUrl, firstRobot) {
  const find = (selector) => root.querySelector(selector);
  const findAll = (selector) => [...root.querySelectorAll(selector)];
  const stage = find('.ix-stage'), view = find('.ix-view3d'), explorer = find('.ix-explorer');
  // robot is the key of the robot that the view shows, which showRobot sets
  const state = { robot: null, layers: new Set(), selected: null, enlarged: false, frame: 0 };

  // --- Scene ---------------------------------------------------------------------------------
  const canvas = document.createElement('canvas');
  canvas.className = 'ix-canvas';
  canvas.setAttribute('role', 'img');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(35, 1.5, 0.01, 10);
  camera.up.set(0, 0, 1);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x777777, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(0.6, -0.8, 1.2);
  scene.add(sun);
  const grid = new THREE.GridHelper(1, 20, 0x888888, 0x888888);  // 5 cm squares on the ground
  grid.rotation.x = Math.PI / 2;
  grid.material.transparent = true;
  grid.material.opacity = 0.35;
  scene.add(grid);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = !reducedMotion();
  controls.target.copy(TARGET);
  // One finger up or down scrolls the page, one finger sideways turns the view
  canvas.style.touchAction = 'pan-y';
  controls.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };

  const label = document.createElement('div');
  label.className = 'ix-frame-label';
  label.hidden = true;
  const hint = document.createElement('div');
  hint.className = 'ix-zoom-hint';
  hint.textContent = 'Hold Ctrl and scroll to zoom';
  hint.hidden = true;

  // Draw only when something changed. With damping, the camera keeps moving for a few frames
  function render() {
    state.frame = 0;
    const moving = controls.update();
    renderer.render(scene, camera);
    placeLabel();
    if (moving && controls.enableDamping) requestRender();
  }
  function requestRender() { if (!state.frame) state.frame = requestAnimationFrame(render); }
  controls.addEventListener('change', requestRender);

  // --- The robot -----------------------------------------------------------------------------
  const gltf = new GLTFLoader();
  const models = new Map();  // glTF file -> promise of its scene
  let robot = null, robotSpec = null;  // The robot that the view shows, and its entry in the data
  let content = { visual: [], collision: [], inertia: [], com: [], frames: [] };
  const linkParts = new Map();  // link name -> its own objects, for the selection box
  const selectionBox = new THREE.Box3Helper(new THREE.Box3(), COLORS.selected);
  selectionBox.material.depthTest = false;
  selectionBox.renderOrder = 20;
  selectionBox.visible = false;
  scene.add(selectionBox);

  function loadModel(file) {
    if (!models.has(file)) {
      const loading = new Promise((resolve, reject) => {
        gltf.load(new URL(`meshes/${file}`, dataUrl).href, (result) => resolve(result.scene), undefined, reject);
      });
      models.set(file, loading);
      // A file that couldn't be loaded isn't kept, so the next robot that needs it asks for it again
      loading.catch(() => models.delete(file));
    }
    return models.get(file);
  }

  const partsOf = (link) => {
    if (!linkParts.has(link.name)) linkParts.set(link.name, []);
    return linkParts.get(link.name);
  };

  // Fetch and read a robot's URDF and its parts' models. This takes a moment and changes nothing
  // in the view, so the robot that is shown stays whole if a file fails or the reader chooses again
  async function loadRobot(key) {
    const spec = data.robots[key];
    const response = await fetch(new URL(spec.urdf, dataUrl));
    if (!response.ok) throw new Error(`${spec.urdf}: ${response.status}`);
    const loader = new URDFLoader();
    loader.parseCollision = true;
    // A mesh keeps its package:// address, which the table of visuals knows
    loader.packages = (name) => `package://${name}`;
    const waiting = [];
    loader.loadMeshCb = (path, manager, material, done) => {
      const file = spec.visuals[meshAddress(path)];
      if (!file) { done(null); return; }  // No model: the link is drawn as its collision shape
      waiting.push(loadModel(file).then((model) => {
        const copy = model.clone(true);
        copy.rotation.x = Math.PI / 2;  // glTF scenes have y up, the robot's frames z up
        const holder = new THREE.Group();
        holder.userData.model = file;
        holder.add(copy);
        done(holder);
      }));
    };
    const model = loader.parse(await response.text());
    await Promise.all(waiting);
    return { key, spec, model };
  }

  // Put a loaded robot into the view, in place of the one that is shown
  function showRobot(loaded) {
    if (robot) scene.remove(robot);
    robot = loaded.model;
    robotSpec = loaded.spec;
    state.robot = loaded.key;
    content = { visual: [], collision: [], inertia: [], com: [], frames: [] };
    linkParts.clear();
    const links = Object.values(robot.links);
    for (const link of links) {
      const visuals = link.children.filter((child) => child.isURDFVisual);
      const colliders = link.children.filter((child) => child.isURDFCollider);
      for (const collider of colliders) {
        collider.traverse((mesh) => {
          if (mesh.isMesh) mesh.material = new THREE.MeshStandardMaterial({ color: COLORS.collision, transparent: true, opacity: 0.55, depthWrite: false, roughness: 0.6 });
        });
        content.collision.push(collider);
        partsOf(link).push(collider);
      }
      for (const visual of visuals) {
        const model = visual.children.find((child) => child.userData.model);
        if (model) {
          // The parts' models are in metres: a scale that the URDF gives its own mesh doesn't apply
          visual.scale.set(1, 1, 1);
          visual.userData.shown = `model: ${data.parts[model.userData.model]}`;
          visual.traverse((mesh) => { if (mesh.isMesh) mesh.material = mesh.material.clone(); });
        } else if (visual.children.length === 0) {
          // No model for this part: its collision shape stands in, in gray
          for (const collider of colliders) {
            const standIn = new THREE.Group();
            standIn.position.copy(collider.position);
            standIn.quaternion.copy(collider.quaternion);
            for (const mesh of collider.children) {
              const copy = mesh.clone();
              copy.material = new THREE.MeshStandardMaterial({ color: COLORS.standIn, roughness: 0.85 });
              standIn.add(copy);
            }
            visual.parent.add(standIn);
            content.visual.push(standIn);
            partsOf(link).push(standIn);
          }
          visual.userData.shown = 'simple shape (the collision shape)';
          continue;
        } else {
          visual.userData.shown = shapeText(visual.urdfNode);
        }
        content.visual.push(visual);
        partsOf(link).push(visual);
      }
      // Inertia: a frame at the center of mass, the equivalent box and a dot
      if (link.inertial && link.inertial.mass > 0) {
        const { xyz, rpy } = link.inertial.origin;
        const frame = new THREE.Group();
        frame.position.set(...xyz);
        frame.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');  // URDF: roll, pitch, yaw about fixed axes
        link.add(frame);
        const size = inertiaBox(link.inertial);
        if (size) {
          const box = new THREE.Mesh(new THREE.BoxGeometry(...size), new THREE.MeshStandardMaterial({ color: COLORS.inertia, transparent: true, opacity: 0.45, depthWrite: false }));
          const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry), new THREE.LineBasicMaterial({ color: COLORS.inertia }));
          box.add(edges);
          frame.add(box);
          content.inertia.push(box);
          partsOf(link).push(box);
        }
        const dot = new THREE.Mesh(new THREE.SphereGeometry(0.004, 16, 12), new THREE.MeshBasicMaterial({ color: COLORS.com, depthTest: false }));
        dot.renderOrder = 10;
        frame.add(dot);
        content.com.push(dot);
      }
      const axes = new THREE.AxesHelper(0.03);
      axes.material.depthTest = false;
      axes.renderOrder = 15;
      link.add(axes);  // On the link itself, so the axes turn with the link
      content.frames.push(axes);
    }
    scene.add(robot);
    scene.updateMatrixWorld(true);
    buildTree();
    showDerived();
  }

  // --- Layers --------------------------------------------------------------------------------
  function applyLayers() {
    const others = [...state.layers].some((layer) => layer !== 'visual');
    for (const [layer, objects] of Object.entries(content)) {
      for (const object of objects) object.visible = state.layers.has(layer);
    }
    // The visual shapes become see-through when another layer is shown inside them
    for (const object of content.visual) {
      object.traverse((mesh) => {
        if (!mesh.isMesh) return;
        const material = mesh.material;
        if (material.transparent !== others) { material.transparent = others; material.needsUpdate = true; }
        material.opacity = others ? 0.16 : 1;
        material.depthWrite = !others;
      });
    }
    for (const box of findAll('[data-layer]')) box.checked = state.layers.has(box.dataset.layer);
    updateSelection();
  }

  function showStep(step) {
    state.layers = new Set(STEPS[step].layers);
    for (const button of findAll('[data-step]')) button.setAttribute('aria-pressed', String(button.dataset.step === step));
    for (const text of findAll('[data-step-text]')) text.hidden = text.dataset.stepText !== step;
    const name = STEPS[step].select;
    state.selected = name && robot.links[name] ? name : null;
    applyLayers();
    showInfo();
  }

  // --- Selection -----------------------------------------------------------------------------
  function updateSelection() {
    const link = state.selected && robot.links[state.selected];
    selectionBox.visible = false;
    label.hidden = true;
    for (const button of findAll('.ix-tree button')) button.setAttribute('aria-pressed', String(button.dataset.link === state.selected));
    if (link) {
      scene.updateMatrixWorld(true);
      const box = new THREE.Box3();
      for (const part of linkParts.get(link.name) || []) if (part.visible) box.expandByObject(part);
      if (box.isEmpty()) box.setFromCenterAndSize(link.getWorldPosition(new THREE.Vector3()), new THREE.Vector3(0.01, 0.01, 0.01));
      selectionBox.box.copy(box);
      selectionBox.visible = true;
      label.textContent = link.name;
      label.hidden = false;
    }
    requestRender();
  }

  function placeLabel() {
    const link = state.selected && robot && robot.links[state.selected];
    if (!link || label.hidden) return;
    const point = link.getWorldPosition(new THREE.Vector3()).project(camera);
    const x = (point.x + 1) / 2 * view.clientWidth, y = (1 - point.y) / 2 * view.clientHeight;
    label.style.left = `${Math.min(Math.max(x + 8, 4), view.clientWidth - label.offsetWidth - 4)}px`;
    label.style.top = `${Math.min(Math.max(y - label.offsetHeight - 6, 4), view.clientHeight - label.offsetHeight - 4)}px`;
  }

  function select(name) {
    state.selected = name;
    updateSelection();
    showInfo();
  }

  function showInfo() {
    const info = find('.ix-info');
    const link = state.selected && robot.links[state.selected];
    if (!link) {
      info.innerHTML = '<p>Choose a link in the list, or click a part in the 3D view.</p>';
      canvas.setAttribute('aria-label', `${robotSpec.name}'s robot model in 3D. The list of links and the tables below give the same values as text.`);
      return;
    }
    const rows = [];
    const position = link.getWorldPosition(new THREE.Vector3());
    rows.push(['Frame', `x ${millimetres(position.x)}, y ${millimetres(position.y)}, z ${millimetres(position.z)} mm from <code>${robot.name}</code>`]);
    const joint = link.parent && link.parent.isURDFJoint ? link.parent : null;
    if (joint) {
      const axis = joint.jointType === 'fixed' ? '' : `, turns about ${joint.axis.toArray().map((value) => +value.toFixed(2)).join(' ')}`;
      rows.push(['Joint', `<code>${joint.name}</code>, ${joint.jointType}${axis}, on <code>${joint.parent.name}</code>`]);
    } else {
      rows.push(['Joint', 'none: this is the root link']);
    }
    const visual = link.children.find((child) => child.isURDFVisual);
    rows.push(['Visual', visual ? visual.userData.shown : 'none']);
    const collider = link.children.find((child) => child.isURDFCollider);
    rows.push(['Collision', collider ? shapeText(collider.urdfNode) : 'none']);
    if (link.inertial && link.inertial.mass > 0) {
      const { mass, inertia, origin } = link.inertial;
      const offset = origin.xyz.some((value) => Math.abs(value) > 1e-9) ? `x ${millimetres(origin.xyz[0])}, y ${millimetres(origin.xyz[1])}, z ${millimetres(origin.xyz[2])} mm from the link's frame` : 'at the link’s frame';
      rows.push(['Mass', `${+(mass * 1000).toFixed(1)} g`], ['Center of mass', offset]);
      const moments = [inertia.ixx, inertia.iyy, inertia.izz].map((value) => value.toExponential(2)).join(', ');
      const size = inertiaBox(link.inertial);
      rows.push(['Inertia', `ixx, iyy, izz: ${moments} kg m²${size ? `. Box: ${size.map(millimetres).join(' × ')} mm` : '. Not drawn as a box.'}`]);
    } else {
      rows.push(['Mass', 'none: a frame without a body']);
    }
    info.innerHTML = `<p class="ix-link-name"><code>${link.name}</code></p><dl>${rows.map(([term, text]) => `<dt>${term}</dt><dd>${text}</dd>`).join('')}</dl>`;
    canvas.setAttribute('aria-label', `${robotSpec.name}'s robot model in 3D, link ${link.name} selected. Its values are in the text next to the view.`);
  }

  // The links as a nested list, in the order of the URDF's tree
  function buildTree() {
    const list = (link) => {
      const children = link.children.filter((child) => child.isURDFJoint).flatMap((joint) => joint.children.filter((child) => child.isURDFLink));
      const inner = children.length ? `<ul>${children.map(list).join('')}</ul>` : '';
      // In a narrow list, a long name goes on at the next line after one of its underscores
      const name = link.name.replaceAll('_', '_<wbr>');
      return `<li><button type="button" data-link="${link.name}" aria-pressed="false">${name}</button>${inner}</li>`;
    };
    find('.ix-tree').innerHTML = `<ul>${list(robot)}</ul>`;
    for (const button of findAll('.ix-tree button')) button.addEventListener('click', () => select(button.dataset.link));
  }

  // Where the midpoint of the drive wheels' axle is, read from the model
  function showDerived() {
    const [left, right] = robotSpec.wheelJoints.map((name) => robot.joints[name].children.find((child) => child.isURDFLink));
    const middle = left.getWorldPosition(new THREE.Vector3()).add(right.getWorldPosition(new THREE.Vector3())).multiplyScalar(0.5);
    const base = robot.links.base_link.getWorldPosition(new THREE.Vector3());
    let text = find('.ix-derived');
    if (!text) {
      text = document.createElement('p');
      text.className = 'ix-derived';
      find('[data-step-text="frames"]').after(text);
      text.dataset.stepText = 'frames';
      text.hidden = find('[data-step-text="frames"]').hidden;
    }
    text.textContent = `In ${robotSpec.name}'s model, the midpoint between the two drive wheels is ${millimetres(middle.x)} mm ahead of base_footprint, `
      + `${millimetres(middle.y)} mm to its left and ${millimetres(middle.z)} mm above the ground. base_link is ${millimetres(base.z)} mm above the ground.`;
  }

  // The robot's files first. If they can't be loaded, nothing on the page has changed yet
  try {
    showRobot(await loadRobot(firstRobot));
  } catch (error) {
    renderer.dispose();
    throw error;
  }

  // --- Controls ------------------------------------------------------------------------------
  // Inside the page the mouse wheel keeps scrolling the page. With Ctrl it zooms the view
  let hintTimer = null;
  view.addEventListener('wheel', (event) => {
    if (state.enlarged || event.ctrlKey || event.metaKey) return;
    event.stopPropagation();
    hint.hidden = false;
    clearTimeout(hintTimer);
    hintTimer = setTimeout(() => { hint.hidden = true; }, 1500);
  }, { capture: true });

  function setView(name) {
    // Apply what is left of a drag's damped motion first, so it doesn't move the camera afterwards
    const damping = controls.enableDamping;
    controls.enableDamping = false; controls.update(); controls.enableDamping = damping;
    camera.position.set(...VIEWS[name]);
    controls.target.copy(TARGET);
    controls.update();
    for (const button of findAll('[data-view]')) button.setAttribute('aria-pressed', String(button.dataset.view === name));
    requestRender();
  }
  for (const button of findAll('[data-view]')) button.addEventListener('click', () => setView(button.dataset.view));
  const unpressViews = () => { for (const button of findAll('[data-view]')) button.setAttribute('aria-pressed', 'false'); };
  for (const button of findAll('[data-zoom]')) {
    button.addEventListener('click', () => {
      const factor = button.dataset.zoom === '1' ? 0.8 : 1.25;
      camera.position.sub(controls.target).multiplyScalar(factor).add(controls.target);
      controls.update(); requestRender();
    });
  }
  for (const button of findAll('[data-turn]')) {
    button.addEventListener('click', () => {
      const angle = Number(button.dataset.turn) * Math.PI / 8;
      camera.position.sub(controls.target).applyAxisAngle(new THREE.Vector3(0, 0, 1), angle).add(controls.target);
      controls.update(); unpressViews(); requestRender();
    });
  }
  controls.addEventListener('start', unpressViews);
  for (const button of findAll('[data-step]')) button.addEventListener('click', () => showStep(button.dataset.step));
  for (const box of findAll('[data-layer]')) {
    box.addEventListener('change', () => {
      if (box.checked) state.layers.add(box.dataset.layer); else state.layers.delete(box.dataset.layer);
      for (const button of findAll('[data-step]')) button.setAttribute('aria-pressed', 'false');
      applyLayers();
    });
  }
  const wheels = find('.ix-wheels');
  function turnWheels() {
    const angle = Number(wheels.value);
    for (const name of robotSpec.wheelJoints) robot.joints[name].setJointValue(THREE.MathUtils.degToRad(angle));
    wheels.closest('.ix-control').querySelector('output').textContent = `${angle}°`;
    wheels.setAttribute('aria-valuetext', `${angle} degrees`);
    updateSelection();
  }
  wheels.addEventListener('input', turnWheels);
  wheels.addEventListener('dblclick', () => { wheels.value = wheels.defaultValue; turnWheels(); });
  find('.ix-reset').addEventListener('click', () => { wheels.value = wheels.defaultValue; turnWheels(); setView('3d'); showStep('visual'); });

  // A click that isn't a drag selects the link of the part under the pointer
  const ray = new THREE.Raycaster();
  let pressed = null;
  canvas.addEventListener('pointerdown', (event) => { pressed = [event.clientX, event.clientY]; });
  canvas.addEventListener('pointerup', (event) => {
    if (!pressed || Math.hypot(event.clientX - pressed[0], event.clientY - pressed[1]) > 5) return;
    const box = canvas.getBoundingClientRect();
    const pointer = new THREE.Vector2((event.clientX - box.left) / box.width * 2 - 1, -(event.clientY - box.top) / box.height * 2 + 1);
    ray.setFromCamera(pointer, camera);
    const visible = [];
    for (const layer of ['visual', 'collision', 'inertia']) {
      for (const object of content[layer]) if (object.visible) object.traverse((mesh) => { if (mesh.isMesh) visible.push(mesh); });
    }
    const hit = ray.intersectObjects(visible, false)[0];
    let link = hit ? hit.object : null;
    while (link && !link.isURDFLink) link = link.parent;
    select(link ? link.name : null);
  });

  // The robot select moves from the start box to the camera buttons
  const robotSelect = find('.ix-robot-select');
  find('.ix-viewbar').prepend(robotSelect.closest('label'));
  // A robot's files take a moment to load, and the reader may choose again meanwhile. Only the
  // latest choice changes the view. An earlier choice whose files arrive later, or fail, is dropped
  let choices = 0;
  robotSelect.addEventListener('change', async () => {
    const choice = ++choices, key = robotSelect.value;
    const status = find('.ix-status');
    if (key === state.robot) { status.hidden = true; return; }  // Back to the robot that is shown
    status.hidden = false;
    status.textContent = `Loading ${data.robots[key].name} …`;
    try {
      const loaded = await loadRobot(key);
      if (choice !== choices) return;
      showRobot(loaded);
      // The new robot's wheels start at the slider's first position, as the slider and its text do
      wheels.value = wheels.defaultValue;
      turnWheels();
      showStep(findAll('[data-step]').find((button) => button.getAttribute('aria-pressed') === 'true')?.dataset.step || 'visual');
      status.hidden = true;
    } catch (error) {
      console.error(error);
      if (choice !== choices) return;
      robotSelect.value = state.robot;
      status.textContent = 'This robot’s files couldn’t be loaded. The other robot is still shown.';
    }
  });

  // Enlarge: the whole stage in a dialog that fills the window, where the wheel zooms directly
  const dialog = document.createElement('dialog');
  dialog.className = 'ix-dialog';
  dialog.setAttribute('aria-label', 'URDF explorer');
  root.append(dialog);
  const home = document.createComment('stage');
  const enlarge = find('.ix-enlarge');
  enlarge.addEventListener('click', () => {
    if (state.enlarged) { dialog.close(); return; }
    stage.before(home);
    dialog.append(stage);
    dialog.showModal();
    state.enlarged = true;
    enlarge.textContent = 'Close';
    resize();
  });
  dialog.addEventListener('close', () => {
    home.replaceWith(stage);
    state.enlarged = false;
    enlarge.textContent = 'Enlarge';
    resize();
  });

  function resize() {
    const width = view.clientWidth, height = view.clientHeight;
    if (!width || !height) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    requestRender();
  }
  const observer = new ResizeObserver(resize);

  canvas.addEventListener('webglcontextlost', (event) => {
    event.preventDefault();
    const status = find('.ix-status');
    status.hidden = false;
    status.textContent = 'The browser stopped the 3D view. Reload the page to open it again.';
  });

  // --- Start ---------------------------------------------------------------------------------
  view.append(canvas, label, hint);
  // The controls listen for the Ctrl key on the canvas's document, which they only find now
  // that the canvas is in the page. Connecting again also resets the touch behavior
  controls.disconnect();
  controls.connect();
  canvas.style.touchAction = 'pan-y';
  explorer.hidden = false;
  find('.ix-view3d-start').hidden = true;
  observer.observe(view);
  resize();
  setView('3d');
  turnWheels();  // The wheels' pose, the angle next to the slider and the spoken angle agree from the start
  showStep('visual');
  render();
  find('.ix-poster').hidden = true;
  find('.ix-view-hint').hidden = false;  // The caption's sentence on how to turn and zoom

  return {
    close() {
      observer.disconnect();
      cancelAnimationFrame(state.frame);
      controls.dispose();
      renderer.dispose();
      canvas.remove(); label.remove(); hint.remove(); dialog.remove();
    },
    // For the tests: the robot's links and joints, the layers and one drawn frame's time
    debug: () => ({ robot, state, content, camera, renderer }),
  };
}
