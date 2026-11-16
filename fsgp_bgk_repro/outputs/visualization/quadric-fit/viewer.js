import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport'), status = $('status');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#eaf0f7');
const camera = new THREE.PerspectiveCamera(43, 1, .005, 250);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias:true, preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = .08;
controls.maxDistance = 100;
scene.add(new THREE.HemisphereLight(0xffffff, 0x889fb7, 2.1));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(-3, -4, 8); scene.add(sun);
const fill = new THREE.DirectionalLight(0xdceaff, .9);
fill.position.set(4, 3, 2); scene.add(fill);
const objects = new THREE.Group(), guides = new THREE.Group();
scene.add(objects, guides);
const state = {ready:false, data:null, manifest:null, selected:null, entries:[], view:'orbit', request:0};
const cache = new Map();
const palette = [0x2879d5, 0x27bfc1, 0xf5cf4a, 0xef7337, 0xba263a].map(x => new THREE.Color(x));
const stops = [0, .3, .62, .82, 1];
const kinds = {quadric:'二次曲面', plane:'平面', distribution:'分布回退'};
const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = .065;

function residualColor(value) {
  if(value === null) return new THREE.Color('#acb7c3');
  const t = Math.min(1, Math.max(0, value/10));
  let i = 0;
  while(i < stops.length-2 && t > stops[i+1]) i++;
  return palette[i].clone().lerp(palette[i+1], (t-stops[i])/(stops[i+1]-stops[i]));
}
function segmentColor(id) {
  return id < 0 ? new THREE.Color('#b5bfcb') : new THREE.Color().setHSL((id*.61803398875)%1, .55, .48);
}
function number(value, digits=2) { return Number.isFinite(value) ? value.toFixed(digits) : '—'; }
function dispose(group) {
  const geometries = new Set(), materials = new Set();
  group.traverse(o => {
    if(o.geometry) geometries.add(o.geometry);
    if(o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => materials.add(m));
  });
  geometries.forEach(g => g.dispose()); materials.forEach(m => m.dispose());
  group.clear();
}
function geometry(positions) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  return g;
}
function pointObject(ids, id) {
  const d = state.data;
  const positions = [], residualColors = [], segmentColors = [];
  for(const index of ids) {
    positions.push(...d.points.slice(index*3, index*3+3));
    residualColor(d.residual_cm[index]).toArray(residualColors, residualColors.length);
    segmentColor(id).toArray(segmentColors, segmentColors.length);
  }
  const g = geometry(positions);
  const p = new THREE.Points(g, new THREE.PointsMaterial({color:'#23394f', size:3.5, sizeAttenuation:false}));
  p.userData = {id, residualColors, segmentColors};
  p.renderOrder = 4;
  objects.add(p);
  return p;
}
function install(data) {
  dispose(objects); dispose(guides);
  state.data = data; state.entries = []; state.selected = null;
  $('patch').replaceChildren(new Option('全部面片', 'all'));
  const indexLists = new Map(data.patches.map(p => [p.id, []]));
  indexLists.set(-1, []);
  data.labels.forEach((id, i) => indexLists.get(id).push(i));
  for(const patch of data.patches) {
    const p = pointObject(indexLists.get(patch.id), patch.id);
    const entry = {patch, points:p, mesh:null, wire:null, lines:null};
    if(patch.vertices.length && patch.triangles.length) {
      const g = geometry(patch.vertices);
      g.setIndex(patch.triangles); g.computeVertexNormals(); g.computeBoundingSphere();
      const material = new THREE.MeshStandardMaterial({
        color:patch.kind==='plane' ? 0x9aa9b9 : 0x409ce1,
        side:THREE.DoubleSide, roughness:.86, metalness:0, transparent:true,
        opacity:1, depthWrite:true, polygonOffset:true, polygonOffsetFactor:1, polygonOffsetUnits:1,
      });
      const mesh = new THREE.Mesh(g, material);
      mesh.userData.id = patch.id; mesh.renderOrder = 1;
      objects.add(mesh); entry.mesh = mesh;
      const wire = new THREE.LineSegments(new THREE.WireframeGeometry(g),
        new THREE.LineBasicMaterial({color:0x2d6f9e, transparent:true, opacity:.25, depthWrite:false}));
      wire.renderOrder = 2; objects.add(wire); entry.wire = wire;
    }
    const lines = new THREE.LineSegments(geometry(patch.projection_lines),
      new THREE.LineBasicMaterial({color:0xe98a29, transparent:true, opacity:.95, depthTest:false}));
    lines.renderOrder = 5; objects.add(lines); entry.lines = lines;
    state.entries.push(entry);
    const suffix = patch.triangles.length ? '' : ' · 无可绘制区域';
    $('patch').add(new Option(`#${patch.id} ${kinds[patch.kind]} · ${patch.count} 点${suffix}`, String(patch.id)));
  }
  if(indexLists.get(-1).length) {
    state.entries.push({patch:{id:-1, kind:'unassigned', count:indexLists.get(-1).length},
      points:pointObject(indexLists.get(-1), -1), mesh:null, wire:null, lines:null});
  }
  const bounds = data.bounds;
  const center = bounds[0].map((x,i) => (x+bounds[1][i])/2);
  const width = Math.max(bounds[1][0]-bounds[0][0], bounds[1][1]-bounds[0][1]);
  const size = Math.ceil(width+1);
  const grid = new THREE.GridHelper(size, size*2, 0xb6c7d8, 0xcad6e3);
  grid.rotation.x = Math.PI/2;
  grid.position.set(center[0], center[1], bounds[0][2]-.06);
  grid.material.transparent = true; grid.material.opacity = .6;
  guides.add(grid);
  const axes = new THREE.AxesHelper(.65);
  axes.position.set(bounds[0][0]-.1, bounds[0][1]-.1, bounds[0][2]-.04); guides.add(axes);
  $('source').href = state.manifest.scenes.find(s => s.id===data.scene).source;
  updateAppearance(); updateInfo(); setView(state.view);
}
function updateAppearance() {
  if(!state.data) return;
  const mode = $('mode').value;
  const opacity = Number($('opacity').value)/100;
  const pointSize = Number($('point-size').value)/10;
  const isolated = $('isolate').checked && state.selected !== null;
  for(const e of state.entries) {
    const chosen = e.patch.id===state.selected;
    const visible = !isolated || chosen;
    e.points.visible = visible && mode!=='surface';
    const mat = e.points.material;
    const attribute = mode==='residual' ? e.points.userData.residualColors :
      mode==='segments' ? e.points.userData.segmentColors : null;
    if(attribute) {
      e.points.geometry.setAttribute('color', new THREE.Float32BufferAttribute(attribute, 3));
      mat.vertexColors = true; mat.color.set(0xffffff);
    } else {
      mat.vertexColors = false; mat.color.set(e.patch.id<0 ? 0xb5bfcb : chosen ? 0x873e16 : 0x23394f);
    }
    mat.size = pointSize; mat.needsUpdate = true;
    if(e.mesh) {
      e.mesh.visible = visible && mode!=='points';
      e.mesh.material.opacity = opacity;
      e.mesh.material.transparent = opacity < 1;
      e.mesh.material.depthWrite = opacity >= 1;
      e.mesh.material.color.copy(chosen && !isolated ? new THREE.Color(0xeea34d) : mode==='segments' ? segmentColor(e.patch.id) :
        new THREE.Color(e.patch.kind==='plane' ? 0x9aa9b9 : 0x409ce1));
      e.wire.visible = e.mesh.visible && $('wireframe').checked;
    }
    if(e.lines) e.lines.visible = visible && $('lines').checked;
  }
  $('colorbar').hidden = mode!=='residual';
  const displayNames={surface:'拟合实面 · 蓝色二次曲面 / 灰色平面',points:'输入点云 · 当前没有显示拟合曲面',
    overlay:'拟合曲面 + 输入点云',residual:'近似距离着色 + 拟合曲面',segments:'分割面片 + 拟合曲面'};
  $('display-label').textContent=(isolated ? `局部面片 #${state.selected} · ` : '')+displayNames[mode];
  document.querySelectorAll('[data-display]').forEach(b => b.classList.toggle('active',b.dataset.display===mode));
  $('opacity-value').textContent = `${Math.round(opacity*100)}%`;
  $('point-value').textContent = `${pointSize.toFixed(1)} px`;
}
function updateInfo() {
  if(!state.data) return;
  const data = state.data, patches = data.patches;
  const q = patches.filter(p => p.kind==='quadric').length, planes = patches.filter(p => p.kind==='plane').length;
  const rows = [[data.point_count.toLocaleString(), '观测点'], [q, '二次曲面'], [planes, '平面'],
    [`${number(data.residual?.rmse_cm)} cm`, 'Taubin 距离 RMSE']];
  $('unassigned-legend').hidden = !data.unassigned_points;
  $('stats').replaceChildren(...rows.map(([value, label]) => {
    const node = document.createElement('div'); node.className = 'metric';
    const strong = document.createElement('strong'); strong.textContent = value;
    node.append(strong, label); return node;
  }));
  const patch = patches.find(p => p.id===state.selected);
  $('patch-info').replaceChildren();
  $('selection-hint').textContent = patch ? `选中面片 #${patch.id} · ${kinds[patch.kind]}` : '点击一个蓝色曲面，或从上方选择面片。';
  if(!patch) return;
  const details = [['观测点', `${patch.count} 个`], ['曲面网格', `${patch.triangles.length/3} 个三角形`],
    ['近似距离 RMSE', `${number(patch.residual?.rmse_cm)} cm`],
    ['近似距离 P95', `${number(patch.residual?.p95_cm)} cm`],
    ['原点 XYZ (m)', patch.origin.map(x => number(x,3)).join(', ')]];
  for(const [name, value] of details) {
    const row = document.createElement('div'); row.className='detail-row';
    const a = document.createElement('span'), b = document.createElement('span');
    a.textContent=name; b.textContent=value; row.append(a,b); $('patch-info').append(row);
  }
  const label=document.createElement('p'); label.className='muted'; label.textContent='系数 a₁…a₁₀（等比归一化）：';
  const coefficients=document.createElement('div'); coefficients.className='coefficients';
  const scale=Math.max(...patch.coefficients.map(Math.abs));
  coefficients.textContent=patch.coefficients.map(c => (scale ? c/scale : c).toPrecision(4)).join(', ');
  $('patch-info').append(label,coefficients);
}
function visibleBounds() {
  if($('isolate').checked && state.selected !== null) {
    const e = state.entries.find(e => e.patch.id===state.selected);
    if(e) {
      e.points.geometry.computeBoundingBox();
      const box=e.points.geometry.boundingBox.clone();
      if(e.mesh) {e.mesh.geometry.computeBoundingBox(); box.union(e.mesh.geometry.boundingBox);}
      return box;
    }
  }
  const box=new THREE.Box3(new THREE.Vector3(...state.data.bounds[0]), new THREE.Vector3(...state.data.bounds[1]));
  for(const e of state.entries) if(e.mesh) {e.mesh.geometry.computeBoundingBox(); box.union(e.mesh.geometry.boundingBox);}
  return box;
}
function setView(view) {
  state.view=view;
  if(!state.data) return;
  const box=visibleBounds(), center=box.getCenter(new THREE.Vector3()), extent=box.getSize(new THREE.Vector3());
  const halfFov=THREE.MathUtils.degToRad(camera.fov/2);
  const limitingFov=Math.min(halfFov,Math.atan(Math.tan(halfFov)*camera.aspect));
  const distance=Math.max(extent.length()/2,.1)/Math.sin(limitingFov)*1.06;
  const direction=new THREE.Vector3(...(view==='top' ? [0,-.0001,1] : view==='side' ? [0,-1,.1] : [.92,-1.12,.82])).normalize();
  camera.position.copy(center).addScaledVector(direction,distance);
  controls.target.copy(center); controls.update();
  document.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('active', b.dataset.view===view));
}
function selectPatch(value) {
  state.selected=value==='all' || value===null ? null : Number(value);
  $('patch').value=state.selected===null ? 'all' : String(state.selected);
  updateAppearance(); updateInfo();
  if($('isolate').checked) setView(state.view);
}
async function json(path) {
  const response=await fetch(path);
  if(!response.ok) throw new Error(`读取失败: ${path} (${response.status})`);
  return response.json();
}
async function loadFrame() {
  const token=++state.request;
  const name=$('scene').value, frame=Number($('frame').value);
  state.ready=false; status.textContent='正在加载点云与拟合曲面…'; status.classList.remove('error');
  try {
    const key=`${name}_${String(frame).padStart(3,'0')}`;
    if(!cache.has(key)) cache.set(key, await json(`./frames/${key}.json`));
    if(token!==state.request) return;
    install(cache.get(key));
    state.ready=true;
    const missing=state.data.patches.filter(p => !p.triangles.length).length;
    status.textContent=`${name}.pcd · 第 ${frame+1}/8 帧 · XYZ 单位 m${missing ? ` · ${missing} 个面片无可绘制区域` : ''}`;
  } catch(error) {
    if(token!==state.request) return;
    status.textContent=error.message; status.classList.add('error'); console.error(error);
  }
}
$('scene').addEventListener('change', loadFrame);
$('frame').addEventListener('change', loadFrame);
$('patch').addEventListener('change', () => selectPatch($('patch').value));
function showDisplay(mode){
  $('mode').value=mode;
  $('opacity').value=mode==='surface' ? 100 : 65;
  updateAppearance();
}
document.querySelectorAll('[data-display]').forEach(button=>button.addEventListener('click',()=>showDisplay(button.dataset.display)));
$('detail').addEventListener('click',()=>{
  if(!state.ready) return;
  const patch=state.data.patches.filter(p=>p.kind==='quadric'&&p.triangles.length).sort((a,b)=>b.count-a.count)[0];
  if(!patch) return;
  $('isolate').checked=true;
  $('wireframe').checked=true;
  showDisplay('surface'); selectPatch(patch.id);
  viewport.scrollIntoView({block:'center',behavior:'smooth'});
});
$('all-patches').addEventListener('click',()=>{
  $('isolate').checked=false; $('wireframe').checked=false;
  selectPatch('all'); setView(state.view);
});
for(const id of ['mode','wireframe','lines','opacity','point-size']) $(id).addEventListener('input', updateAppearance);
$('isolate').addEventListener('change', () => { updateAppearance(); setView(state.view); });
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => setView(b.dataset.view)));
let down=null;
renderer.domElement.addEventListener('pointerdown', event => { down=[event.clientX,event.clientY]; });
renderer.domElement.addEventListener('pointerup', event => {
  if(!state.ready || !down || Math.hypot(event.clientX-down[0],event.clientY-down[1])>5) return;
  const rect=renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1), camera);
  const meshes=state.entries.map(e => e.mesh).filter(o => o?.visible);
  const points=state.entries.map(e => e.points).filter(o => o.visible && o.userData.id>=0);
  const hits=raycaster.intersectObjects(meshes.length ? meshes : points, false);
  if(hits.length) selectPatch(hits[0].object.userData.id);
});
new ResizeObserver(() => {
  const width=viewport.clientWidth,height=viewport.clientHeight;
  camera.aspect=width/height; camera.updateProjectionMatrix(); renderer.setSize(width,height);
  if(state.data) setView(state.view);
  renderer.render(scene,camera);
}).observe(viewport);
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene,camera); });
window.quadricFitViewer={state,scene,camera,renderer,controls,setView,selectPatch,loadFrame,updateAppearance};
try {
  state.manifest=await json('./manifest.json');
  for(const s of state.manifest.scenes) $('scene').add(new Option(`${s.name} · ${s.id}`,s.id));
  for(let i=0;i<8;i++) $('frame').add(new Option(`第 ${i+1} / 8 帧`,String(i)));
  $('scene').value=state.manifest.default_scene; $('frame').value=String(state.manifest.default_frame);
  await loadFrame();
} catch(error) { status.textContent=error.message; status.classList.add('error'); console.error(error); }
