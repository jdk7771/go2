import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// Rendering consumes exported data only; no planning or collision scoring runs here.
const manifest = window.PLANNER_MANIFEST;
const $ = id => document.getElementById(id);
const state = { data: null, mode: 'full', phase: 'compare', frame: 0,
  node: 11, playing: false, started: 0, startFrame: 0, request: 0, pose: null };
const color = { initial: 0xdf8b38, final: 0x2189b8, gold: 0xf4c34d, nose: 0xdc6656 };
const viewport = $('viewport');
const scene = new THREE.Scene();
const camera = new THREE.OrthographicCamera(-2, 2, 1.3, -1.3, 0.01, 100);
camera.up.set(0, 0, 1);
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true });
} catch (error) {
  showError('无法启动三维画面，请使用启用了 WebGL 的 Chrome、Edge 或 Firefox。');
  throw error;
}
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0xf1f6f9, 0);
viewport.prepend(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = false;
controls.minDistance = 0.18;
controls.maxDistance = 20;
controls.minZoom = 0.2;
controls.maxZoom = 20;
controls.target.set(0, 0, 0.35);
scene.add(new THREE.AmbientLight(0xffffff, 2.3));
const light = new THREE.DirectionalLight(0xffffff, 2.6);
light.position.set(1, -2, 4);
scene.add(light);
const cylinderGeometry = new THREE.CylinderGeometry(1, 1, 1, 7);
const grid = new THREE.GridHelper(4, 20, 0xb9ccd8, 0xd6e1e8);
grid.rotation.x = Math.PI / 2;
grid.scale.z = 0.5;
grid.material.transparent = true;
grid.material.opacity = 0.5;
scene.add(grid);
const worldAxes = new THREE.AxesHelper(0.24);
worldAxes.position.set(-1.65, -0.68, 0.01);
scene.add(worldAxes);
let content, cloudPoints, floorPoints, initialPath, currentPath, body, initialBody, ghosts, crossSection;
const matrix = new THREE.Matrix4(), quaternion = new THREE.Quaternion(), vector = new THREE.Vector3();
const up = new THREE.Vector3(0, 1, 0);

function showError(message) {
  $('error').textContent = message;
  $('error').hidden = false;
  $('loading').hidden = true;
}

function removeContent() {
  if (!content) return;
  const geometries = new Set(), materials = new Set();
  content.traverse(object => {
    if (object.geometry && object.geometry !== cylinderGeometry) geometries.add(object.geometry);
    for (const material of (Array.isArray(object.material) ? object.material : [object.material])) {
      if (material) materials.add(material);
    }
  });
  geometries.forEach(geometry => geometry.dispose());
  materials.forEach(material => material.dispose());
  scene.remove(content);
}

function makePoints(points, opacity, size, floor = false) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(points.flat(), 3));
  const colors = [];
  const low = new THREE.Color(0x80a4b8), high = new THREE.Color(0x3e7189);
  const temporary = new THREE.Color();
  for (const point of points) {
    temporary.lerpColors(low, high, Math.max(0, Math.min(1, point[2] / 0.7)));
    colors.push(temporary.r, temporary.g, temporary.b);
  }
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  return new THREE.Points(geometry, new THREE.PointsMaterial({ size, sizeAttenuation: false,
    transparent: true, opacity, depthWrite: false, vertexColors: !floor, color: floor ? 0xabbecb : 0xffffff }));
}

function wireBody(half, tint, opacity = 1) {
  const geometry = new THREE.BoxGeometry(...half.map(n => 2 * n));
  const edges = new THREE.EdgesGeometry(geometry);
  geometry.dispose();
  return new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: tint,
    transparent: true, opacity, depthWrite: false }));
}

function solidBody(half) {
  const group = new THREE.Group();
  const materials = [color.nose, 0x377dac, 0x378fc3, 0x85b9d9, color.gold, 0x204e72]
    .map(tint => new THREE.MeshStandardMaterial({ color: tint, roughness: 0.65, metalness: 0.03 }));
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...half.map(n => 2*n)), materials);
  group.add(mesh, wireBody(half, 0x193047));
  const chevron = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-0.05, -half[1] * 0.55, half[2] + 0.002),
    new THREE.Vector3(half[0] * 0.70, 0, half[2] + 0.002),
    new THREE.Vector3(-0.05, half[1] * 0.55, half[2] + 0.002),
  ]);
  group.add(new THREE.Line(chevron, new THREE.LineBasicMaterial({ color: 0xffffff })));
  for (let i = 0; i < 3; i++) {
    const direction = new THREE.Vector3().setComponent(i, 1);
    group.add(new THREE.ArrowHelper(direction, new THREE.Vector3(), half[i] + 0.08,
      [color.nose, 0x12856e, 0x7860c3][i], 0.045, 0.022));
  }
  return group;
}

function setPose(object, pose) {
  object.position.fromArray(pose);
  // Matches the Python planner exactly: R = Rz(yaw) Ry(pitch) Rx(roll).
  object.rotation.set(pose[3], pose[4], pose[5], 'ZYX');
  object.updateMatrix();
}

function makePath(poses) {
  const mesh = new THREE.InstancedMesh(cylinderGeometry,
    new THREE.MeshBasicMaterial({ color: color.final }), poses.length - 1);
  mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  updatePath(mesh, poses);
  return mesh;
}

function updatePath(mesh, poses) {
  for (let i = 0; i < poses.length - 1; i++) {
    const a = new THREE.Vector3(...poses[i].slice(0, 3));
    const b = new THREE.Vector3(...poses[i+1].slice(0, 3));
    const direction = b.clone().sub(a);
    const length = direction.length();
    quaternion.setFromUnitVectors(up, direction.normalize());
    matrix.compose(a.add(b).multiplyScalar(0.5), quaternion, vector.set(0.004, length, 0.004));
    mesh.setMatrixAt(i, matrix);
  }
  mesh.instanceMatrix.needsUpdate = true;
  mesh.computeBoundingSphere();
}

function buildContent() {
  removeContent();
  content = new THREE.Group();
  const data = state.data;
  const obstacles = data.cloud.filter(point => point[2] > 0.035);
  const floor = data.cloud.filter(point => point[2] <= 0.035);
  cloudPoints = makePoints(obstacles, 0.28, 3.1);
  floorPoints = makePoints(floor, 0.05, 2, true);
  crossSection = new THREE.Group();
  for (const segment of data.crossSection || []) {
    const points = segment.yz.map(([y, z]) => new THREE.Vector3(0, y, z));
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    crossSection.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({
      color: segment.color, depthTest: false, transparent: true, opacity: 0.9,
    })));
    for (const point of points) {
      const dot = new THREE.Mesh(new THREE.SphereGeometry(0.005, 8, 6),
        new THREE.MeshBasicMaterial({ color: segment.color, depthTest: false }));
      dot.position.copy(point); crossSection.add(dot);
    }
  }
  crossSection.renderOrder = 20;
  const initialGeometry = new THREE.BufferGeometry().setFromPoints(data.initial.map(p => new THREE.Vector3(...p.slice(0, 3))));
  initialPath = new THREE.Line(initialGeometry, new THREE.LineDashedMaterial({ color: color.initial, dashSize: 0.045, gapSize: 0.025 }));
  initialPath.computeLineDistances();
  currentPath = makePath(trial().final);
  body = solidBody(data.half);
  initialBody = wireBody(data.half, color.initial, 0.95);
  ghosts = new THREE.Group();
  for (let i = 0; i < 6; i++) ghosts.add(wireBody(data.half, color.final, 0.32));
  content.add(cloudPoints, floorPoints, initialPath, currentPath, body, initialBody, ghosts, crossSection);
  scene.add(content);
  updateVisibility();
  updateOpacity();
}

function trial() { return state.data.trials[state.mode]; }
function lastFrame() { return state.phase === 'optimize' ? Math.max(0, trial().history.length-1) : trial().dense.length-1; }
function degrees(angle) { return `${angle >= 0 ? '+' : ''}${(angle*180/Math.PI).toFixed(1)}°`; }
function interpolate(poses, fraction) {
  const index = Math.max(0, Math.min(poses.length-1, fraction*(poses.length-1)));
  const left = Math.min(Math.floor(index), poses.length-2), blend = index-left;
  return poses[left].map((value, i) => value*(1-blend)+poses[left+1][i]*blend);
}

function updateFrame() {
  if (!state.data) return;
  const selected = trial();
  const optimizing = state.phase === 'optimize' && selected.history.length;
  const poses = optimizing ? selected.history[state.frame] : selected.final;
  const fraction = state.frame / Math.max(1, lastFrame());
  const pose = optimizing ? poses[state.node] : selected.dense[state.frame];
  state.pose = pose;
  setPose(body, pose);
  setPose(initialBody, optimizing ? state.data.initial[state.node] : interpolate(state.data.initial, fraction));
  updatePath(currentPath, poses);
  ghosts.children.forEach((ghost, i) => setPose(ghost, poses[Math.round((i+1)*(poses.length-1)/7)]));
  ['roll', 'pitch', 'yaw'].forEach((id, i) => { $(id).textContent = degrees(pose[3+i]); });
  $('position').textContent = `x ${pose[0].toFixed(3)}  y ${pose[1].toFixed(3)}  z ${pose[2].toFixed(3)} m`;
  $('timeline').max = lastFrame();
  $('timeline').value = state.frame;
  $('timeline-label').textContent = optimizing ? `迭代 ${state.frame} / ${lastFrame()}` : `路径 ${(fraction*100).toFixed(0)}% · x ${pose[0].toFixed(2)} m`;
  $('phase-note').textContent = optimizing
    ? `真实优化迭代 · 观察第 ${state.node+1} 个节点 · 目标值 ${selected.costs[state.frame].toPrecision(4)}`
    : state.phase === 'compare' ? '橙色虚线：初始路径 · 蓝色实线：优化后路径 · 可拖动进度检查任意位置' : '沿最终轨迹回放 · 姿态与位置同步变化';
  $('local-clearance').textContent = optimizing ? '' : `当前位姿真值间隙 ${(selected.clearance[state.frame]*1000).toFixed(1)} mm`;
  const node = optimizing ? state.node : Math.round(fraction*(state.data.initial.length-1));
  $('node').value = node;
  $('node-value').textContent = `${node+1} / ${state.data.initial.length}`;
  drawChart(poses, pose);
}

function drawChart(poses, pose) {
  const canvas = $('angle-chart'), ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height, pad = 12;
  ctx.clearRect(0, 0, w, h);
  const x0 = state.data.initial[0][0], x1 = state.data.initial.at(-1)[0];
  const limit = Math.max(0.26, ...poses.flatMap(p => p.slice(3).map(Math.abs))) * 1.15;
  const x = value => pad+(value-x0)/(x1-x0)*(w-2*pad);
  const y = value => h/2-value/limit*(h/2-pad);
  ctx.strokeStyle = '#dce5ed'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(pad, h/2); ctx.lineTo(w-pad, h/2); ctx.stroke();
  ['#12856e', '#b97619', '#7860c3'].forEach((tint, i) => {
    ctx.strokeStyle = tint; ctx.lineWidth = 3; ctx.beginPath();
    poses.forEach((p, n) => n ? ctx.lineTo(x(p[0]), y(p[i+3])) : ctx.moveTo(x(p[0]), y(p[i+3])));
    ctx.stroke(); ctx.fillStyle = tint; ctx.beginPath(); ctx.arc(x(pose[0]), y(pose[i+3]), 4, 0, 2*Math.PI); ctx.fill();
  });
  ctx.strokeStyle = '#8a9eae'; ctx.setLineDash([4, 5]); ctx.beginPath();
  ctx.moveTo(x(pose[0]), pad); ctx.lineTo(x(pose[0]), h-pad); ctx.stroke(); ctx.setLineDash([]);
}

function updateVisibility() {
  if (!content) return;
  cloudPoints.visible = floorPoints.visible = $('show-cloud').checked;
  initialPath.visible = initialBody.visible = $('show-initial').checked;
  currentPath.visible = body.visible = $('show-final').checked;
  ghosts.visible = $('show-ghosts').checked && $('show-final').checked;
  crossSection.visible = $('show-section').checked;
}
function updateOpacity() {
  const opacity = Number($('opacity').value)/100;
  $('opacity-value').value = `${Math.round(opacity*100)}%`;
  if (cloudPoints) { cloudPoints.material.opacity = opacity; floorPoints.material.opacity = opacity*0.18; }
}
function setPlaying(playing) {
  state.playing = playing;
  if (playing && state.frame >= lastFrame()) state.frame = 0;
  state.started = performance.now(); state.startFrame = state.frame;
  $('play').textContent = playing ? 'Ⅱ' : '▶';
  $('play').setAttribute('aria-label', playing ? '暂停' : '播放');
}
function setPhase(phase) {
  if (phase === 'optimize' && !trial().history.length) return;
  setPlaying(false); state.phase = phase;
  state.frame = phase === 'compare' ? Math.round((trial().dense.length-1)/2) : 0;
  document.querySelectorAll('[data-phase]').forEach(button => button.classList.toggle('active', button.dataset.phase === phase));
  updateFrame();
}
function setMode(mode) {
  setPlaying(false); state.mode = mode; $('method').value = mode;
  const value = trial().metrics.minimum_signed_distance_m;
  const classification = value < 0 ? 'danger' : value < 0.020 ? 'caution' : 'safe';
  $('minimum').textContent = `${(value*1000).toFixed(2)} mm`;
  $('minimum').className = $('safety-label').className = classification;
  $('safety-label').textContent = value < 0 ? '存在碰撞' : value < 0.020 ? '未达到 20 mm 目标' : '达到 20 mm 间隙目标';
  const hasHistory = !!trial().history.length;
  document.querySelector('[data-phase="optimize"]').disabled = !hasHistory;
  document.querySelector('[data-phase="optimize"]').title = hasHistory ? '' : '该场景未记录优化过程，可查看初始/最终轨迹';
  if (!hasHistory && state.phase === 'optimize') state.phase = 'compare';
  setPhase(state.phase);
}
function setView(view = 'iso', zoom = 1) {
  const target = new THREE.Vector3(0, 0, 0.35);
  const offsets = { iso: [2.7, -3.6, 2.3], front: [4.1, 0, 0], side: [0, -4.1, 0], top: [0, -0.001, 4.1] };
  controls.target.copy(target); camera.position.copy(target).add(new THREE.Vector3(...offsets[view]));
  camera.zoom = zoom; camera.updateProjectionMatrix(); controls.update();
}
function focusBody() {
  if (!state.pose) return;
  const direction = camera.position.clone().sub(controls.target).normalize();
  controls.target.fromArray(state.pose);
  camera.position.copy(controls.target).addScaledVector(direction, 4.1);
  camera.zoom = 2.4; camera.updateProjectionMatrix();
  controls.update();
}

function loadPayload(entry) {
  if (window.PLANNER_SCENES?.[entry.id]) return Promise.resolve(window.PLANNER_SCENES[entry.id]);
  return new Promise((resolve, reject) => {
    const script = document.createElement('script'); script.src = entry.file;
    script.onload = () => { script.remove(); resolve(window.PLANNER_SCENES[entry.id]); };
    script.onerror = () => { script.remove(); reject(new Error(`场景数据未找到：${entry.label}`)); };
    document.head.append(script);
  });
}
async function setScene(name) {
  const request = ++state.request;
  setPlaying(false); $('loading').hidden = false; $('error').hidden = true;
  try {
    const entry = manifest.scenes.find(item => item.id === name);
    const data = await loadPayload(entry);
    if (request !== state.request) return;
    state.data = data; state.node = Math.floor(data.initial.length/2);
    for (const option of $('method').options) option.disabled = !data.trials[option.value];
    if (!data.trials[state.mode]) state.mode = Object.keys(data.trials)[0];
    $('scene').value = entry.sourceScene;
    $('dataset').replaceChildren();
    manifest.scenes.filter(item => item.sourceScene === entry.sourceScene).forEach(item => {
      $('dataset').add(new Option(item.selectionLabel || item.run, item.id));
    });
    $('dataset').value = name;
    $('scene-name').textContent = data.label;
    $('run-name').textContent = data.sourceRun || manifest.run;
    $('report-link').href = data.report || 'report/REPORT.md';
    $('scene-description').textContent = data.description || '';
    $('scene-description').hidden = !data.description;
    $('section-toggle').hidden = $('section-key').hidden = !data.crossSection?.length;
    $('node').max = data.initial.length-1;
    buildContent(); setMode(state.mode);
    setView(data.defaultView || 'iso', data.defaultZoom || 1);
    $('loading').hidden = true;
  } catch (error) { console.error(error); showError(error.message); }
}

$('run-name').textContent = manifest.run;
const sceneNames = new Set();
manifest.scenes.forEach(entry => {
  if (!sceneNames.has(entry.sourceScene)) {
    $('scene').add(new Option(entry.sceneLabel || entry.label, entry.sourceScene));
    sceneNames.add(entry.sourceScene);
  }
});
$('scene').onchange = event => {
  const current = manifest.scenes.find(item => item.id === state.data?.name);
  const available = manifest.scenes.filter(item => item.sourceScene === event.target.value);
  setScene((available.find(item => item.run === current?.run) || available[0]).id);
};
$('dataset').onchange = event => setScene(event.target.value);
$('method').onchange = event => setMode(event.target.value);
document.querySelectorAll('[data-phase]').forEach(button => button.onclick = () => setPhase(button.dataset.phase));
document.querySelectorAll('[data-view]').forEach(button => button.onclick = () => setView(button.dataset.view));
['show-cloud', 'show-initial', 'show-final', 'show-ghosts', 'show-section'].forEach(id => { $(id).onchange = updateVisibility; });
$('opacity').oninput = updateOpacity;
$('focus-body').onclick = focusBody; $('reset-view').onclick = () => setView('iso');
$('play').onclick = () => setPlaying(!state.playing);
$('speed').onchange = () => { if (state.playing) setPlaying(true); };
$('timeline').oninput = event => { setPlaying(false); state.frame = Number(event.target.value); updateFrame(); };
$('node').oninput = event => {
  state.node = Number(event.target.value);
  if (state.phase !== 'optimize') { setPlaying(false); state.frame = Math.round(state.node/(state.data.initial.length-1)*lastFrame()); }
  updateFrame();
};
document.addEventListener('keydown', event => {
  if (event.code === 'Space' && !['INPUT', 'SELECT', 'BUTTON'].includes(document.activeElement.tagName) && !$('videos').open) {
    event.preventDefault(); setPlaying(!state.playing);
  }
});
const resize = new ResizeObserver(() => {
  const aspect = viewport.clientWidth/viewport.clientHeight;
  camera.left = -1.3*aspect; camera.right = 1.3*aspect;
  camera.top = 1.3; camera.bottom = -1.3;
  camera.updateProjectionMatrix(); renderer.setSize(viewport.clientWidth, viewport.clientHeight);
});
resize.observe(viewport);
manifest.videos.forEach(entry => $('video-select').add(new Option(entry.label, entry.file)));
$('video-open').hidden = manifest.videos.length === 0;
$('video-open').onclick = () => {
  setPlaying(false);
  const selected = manifest.videos.find(entry => entry.scene === state.data.name) || manifest.videos[0];
  $('video-select').value = selected.file; $('video-player').src = selected.file;
  $('videos').showModal(); $('video-player').play().catch(() => {});
};
$('video-select').onchange = event => { $('video-player').src = event.target.value; $('video-player').play().catch(() => {}); };
$('video-close').onclick = () => $('videos').close();
$('videos').onclose = () => $('video-player').pause();

function animate(now) {
  if (state.playing && state.data) {
    const framesPerSecond = lastFrame()/(state.phase === 'optimize' ? 10 : 14);
    const frame = Math.min(lastFrame(), state.startFrame+Math.floor((now-state.started)/1000*framesPerSecond*Number($('speed').value)));
    if (frame !== state.frame) { state.frame = frame; updateFrame(); }
    if (frame === lastFrame()) setPlaying(false);
  }
  controls.update(); renderer.render(scene, camera); requestAnimationFrame(animate);
}

// A small public inspection API also makes numeric/browser regression checks possible.
window.plannerViewer = {
  setScene, setMode, setPhase, setView, focusBody,
  setProgress(frame) { setPlaying(false); state.frame = Math.max(0, Math.min(lastFrame(), frame)); updateFrame(); },
  getState() { return { ready: !!state.data, scene: state.data?.name, mode: state.mode, phase: state.phase,
    frame: state.frame, pose: state.pose, bodyMatrix: body?.matrix.toArray(),
    camera: camera.position.toArray(), zoom: camera.zoom, target: controls.target.toArray(),
    cloudVisible: cloudPoints?.visible, initialVisible: initialPath?.visible,
    finalVisible: currentPath?.visible, sectionVisible: crossSection?.visible,
    crossSection: state.data?.crossSection || [], playing: state.playing }; },
};
setView('iso'); requestAnimationFrame(animate);
await setScene(manifest.scenes[0].id);
