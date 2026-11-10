// The URDF explorer on the Robot Description page: the part that every visitor loads.
// It only shows the button. three.js, the URDF loader and the robot's files are fetched when the
// reader opens the 3D view (urdf-explorer-view.js). Until then, and if the 3D view can't start,
// the page shows the picture and the tables that it came with.

function prepare(root) {
  const data = JSON.parse(root.querySelector('.ix-data').textContent);
  const find = (selector) => root.querySelector(selector);
  const select = find('.ix-robot-select'), open = find('.ix-open'), status = find('.ix-status');
  const sizes = JSON.parse(find('.ix-download').dataset.sizes);
  const showSize = () => { find('.ix-download').textContent = `Loads ${sizes[select.value]}`; };
  select.addEventListener('change', showSize);
  showSize();
  find('.ix-view3d-start').hidden = false;

  let view = null;
  open.addEventListener('click', async () => {
    open.disabled = true;
    status.hidden = false;
    status.textContent = 'Loading the 3D view …';
    try {
      // The address of the robots' files, which main.html gives for this page
      const dataUrl = new URL(document.querySelector('meta[name="ix-urdf-explorer"]').content, document.baseURI);
      const { openExplorer } = await import('./urdf-explorer-view.js');
      view = await openExplorer(root, data, dataUrl, select.value);
      root.ixView = view;  // For the tests
      status.hidden = true;
      root.classList.add('ix-is-live');
      // The button is gone now: the keyboard's place moves to the first step
      root.querySelector('[data-step]').focus({ preventScroll: true });
    } catch (error) {
      console.error('The 3D view could not start:', error);
      if (view) view.close();
      root.classList.remove('ix-is-live');
      find('.ix-explorer').hidden = true;
      open.disabled = false;
      status.textContent = 'The 3D view couldn’t start. It needs WebGL in the browser and its files from this site. '
        + 'The picture and the tables below show the same models.';
    }
  });
}

function prepareAll() {
  for (const root of document.querySelectorAll('[data-ix="urdf-explorer"]')) prepare(root);
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', prepareAll);
else prepareAll();
