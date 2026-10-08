// Renders build/jobs.json with headless Chromium and writes the JPEGs into docs/.
// Usage: INSIDERS=<path to remo_description_insiders> node render.mjs [name filter]
import { createServer } from 'node:http';
import { readFileSync, writeFileSync, existsSync, realpathSync, statSync } from 'node:fs';
import { join, dirname, extname, resolve, relative, isAbsolute } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright-core';

const here = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(here, '../..');
const insiders = resolve(process.env.INSIDERS || join(repoRoot, '../remo_description_insiders'));
if (!existsSync(join(insiders, 'meshes'))) throw new Error(`no meshes in ${insiders}; set INSIDERS`);
const roots = { '/insiders/': insiders, '/node_modules/': join(here, 'node_modules'), '/build/': join(here, 'build'), '/': here };
const types = { '.html': 'text/html', '.js': 'text/javascript', '.json': 'application/json' };
// Serve a file only if it resolves (symlinks included) to a regular file inside its root.
function inside(root, file) {
  if (!existsSync(root) || !existsSync(file)) return null;
  const real = realpathSync(file), rel = relative(realpathSync(root), real);
  return rel && !rel.startsWith('..') && !isAbsolute(rel) && statSync(real).isFile() ? real : null;
}
const server = createServer((req, res) => {
  let path;
  try { path = decodeURIComponent(new URL(req.url, 'http://x').pathname); } catch { res.writeHead(400).end(); return; }
  const prefix = Object.keys(roots).find(p => path.startsWith(p));
  const file = inside(roots[prefix], join(roots[prefix], path.slice(prefix.length)));
  if (!file) { res.writeHead(404).end(); return; }
  res.writeHead(200, { 'content-type': types[extname(file)] || 'application/octet-stream' }).end(readFileSync(file));
}).listen(0, '127.0.0.1');
await new Promise(ok => server.once('listening', ok));

const filter = process.argv[2] || '';
const jobs = JSON.parse(readFileSync(join(here, 'build/jobs.json'), 'utf8')).filter(j => j.out.includes(filter));
const browser = await chromium.launch({ executablePath: process.env.CHROME_PATH || undefined,
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
page.on('pageerror', e => console.error('page error:', e.message));
await page.goto(`http://127.0.0.1:${server.address().port}/render.html`);
await page.waitForFunction(() => window.ready === true);
for (const job of jobs) {
  const url = await page.evaluate(cfg => window.renderJob(cfg), job.cfg);
  writeFileSync(join(repoRoot, job.out), Buffer.from(url.split(',')[1], 'base64'));
  console.log(job.out);
}
await browser.close();
server.close();
