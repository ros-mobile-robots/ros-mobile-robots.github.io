# STL renders

Renders Remo's STL files into the images on the hardware setup pages:

- **Assembly steps 16 to 22:** `docs/hardware_setup/images/decks/`
- **One render per printed part for the 3D printing page:** `docs/hardware_setup/images/3d_print/`

The parts are placed with the poses from Remo's URDF (`remo.urdf.xacro`). The STL files are only in the private
`remo_description_insiders` repository (Git LFS); the public `remo_description` has empty placeholders.

## Setup

```bash
git clone git@github.com:ros-mobile-robots/remo_description_insiders.git ../remo_description_insiders
git -C ../remo_description_insiders lfs pull
cd scripts/stl-renders
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
npm install
npx playwright install chromium   # or set CHROME_PATH to an existing Chromium
```

## Render

```bash
.venv/bin/python scene.py ../../../remo_description_insiders   # URDF -> build/scene_*.json
.venv/bin/python jobs.py                                       # build/jobs.json
INSIDERS=../../../remo_description_insiders node render.mjs    # all images; add a filter, e.g. "22-camera"
```

The files work like this:

- **`jobs.py`:** for each image, it sets which parts are shown, which are lifted off ("explode"), the screws and guide lines, and the view direction.
- **`holes.py`:** finds hole centres in a part by slicing it. That's how the screw positions in `jobs.py` were found.
- **`render.html`:** draws a job with three.js in headless Chromium.
